/*
 * car.c - Controls one elevator car.
 *
 * Usage: ./car {name} {delay} {lowest floor} {highest floor}
 *
 * Threads (all private state below is protected by the shared mutex, and
 * every thread waits on the shared condition variable):
 *   main      - door and movement state machine, passenger requests and
 *               the safety heartbeat watchdog
 *   network   - connects to the controller and sends queued messages
 *               (never holds the mutex while blocked on the network)
 *   receiver  - reads FLOOR commands from the controller
 *   signal    - waits for SIGINT with sigwait() and starts shutdown
 */

#include "shared.h"

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define WATCHDOG_MS 500
#define QUEUE_SIZE 32
#define MSG_SIZE 64

static struct
{
  /* Fixed after start-up */
  car_shared_mem *shm;
  char name[32];
  char shm_name[64];
  char lowest_text[4];
  char highest_text[4];
  int lowest;
  int highest;
  long delay_ms;

  /* State machine (main thread only) */
  char phase[9];              /* status the phase timer belongs to */
  struct timespec phase_end;  /* when the current phase finishes */
  bool moving;
  int moving_to;              /* floor being travelled to while Between */

  /* Shared between threads - protected by shm->mutex */
  bool shutting_down;
  bool has_floor_cmd;         /* FLOOR received, not yet applied */
  int floor_cmd;
  uint32_t last_heartbeat;
  bool heartbeat_seen;
  struct timespec heartbeat_time;
  bool connected;
  int fd;
  bool receiver_done;
  bool mode_announced;        /* SERVICE/EMERGENCY queued for this connection */
  char last_status_msg[MSG_SIZE];
  char queue[QUEUE_SIZE][MSG_SIZE];
  int queue_head;
  int queue_count;
} car;

static pthread_t receiver_thread;

/* ------------------------------------------------------------- time helpers */

static void now_plus_ms(struct timespec *t, long ms)
{
  clock_gettime(CLOCK_MONOTONIC, t);
  t->tv_sec += ms / 1000;
  t->tv_nsec += (ms % 1000) * 1000000L;
  if (t->tv_nsec >= 1000000000L) {
    t->tv_nsec -= 1000000000L;
    t->tv_sec++;
  }
}

static void add_ms(struct timespec *t, long ms)
{
  t->tv_sec += ms / 1000;
  t->tv_nsec += (ms % 1000) * 1000000L;
  if (t->tv_nsec >= 1000000000L) {
    t->tv_nsec -= 1000000000L;
    t->tv_sec++;
  }
}

static bool time_reached(const struct timespec *now, const struct timespec *t)
{
  return now->tv_sec > t->tv_sec || (now->tv_sec == t->tv_sec && now->tv_nsec >= t->tv_nsec);
}

static const struct timespec *earlier(const struct timespec *a, const struct timespec *b)
{
  if (a == NULL) {
    return b;
  }
  if (b == NULL) {
    return a;
  }
  return time_reached(a, b) ? b : a;
}

/* --------------------------------------------------------- shared memory */

/* Fields in shared memory may be written by other processes and are not
   guaranteed to be NUL-terminated, so always copy them before parsing. */
static bool read_floor_field(const char field[4], int *floor)
{
  char text[5];

  memcpy(text, field, 4);
  text[4] = '\0';
  return floor_to_int(text, floor);
}

static void read_status(char out[9])
{
  memcpy(out, car.shm->status, 8);
  out[8] = '\0';
}

static bool in_range(int floor)
{
  return floor >= car.lowest && floor <= car.highest;
}

static int current_floor(void)
{
  int floor = car.lowest;

  (void)read_floor_field(car.shm->current_floor, &floor);
  return floor;
}

static void set_current(int floor)
{
  int_to_floor(floor, car.shm->current_floor);
}

static void set_target(int floor)
{
  int_to_floor(floor, car.shm->target_floor);
}

static bool in_mode(void)
{
  return car.shm->service_mode == 1 || car.shm->emergency_mode == 1;
}

/* Valid, in-range passenger request waiting in the mailbox */
static bool valid_request(int *floor)
{
  return car.shm->floor_request_pending == 1 &&
         read_floor_field(car.shm->floor_request, floor) && in_range(*floor);
}

/* ---------------------------------------------------------- message queue */

static void enqueue(const char *msg)
{
  if (car.queue_count < QUEUE_SIZE) {
    int tail = (car.queue_head + car.queue_count) % QUEUE_SIZE;
    snprintf(car.queue[tail], MSG_SIZE, "%s", msg);
    car.queue_count++;
  }
  /* A full queue means the controller has stopped reading; the message is
     dropped rather than blocking the car. No broadcast is needed here: the
     main thread broadcasts after any state change, and the network thread
     (the only reader) queues its own messages. */
}

static void format_status(char msg[MSG_SIZE])
{
  char status[9];
  char current[5];
  char target[5];

  read_status(status);
  memcpy(current, car.shm->current_floor, 4);
  current[4] = '\0';
  memcpy(target, car.shm->target_floor, 4);
  target[4] = '\0';
  snprintf(msg, MSG_SIZE, "STATUS %s %s %s", current, target, status);
}

/* Queue a STATUS message if the state differs from the last one queued */
static void queue_status_if_changed(void)
{
  char msg[MSG_SIZE];

  if (car.connected && !car.mode_announced && !in_mode()) {
    format_status(msg);
    if (strcmp(msg, car.last_status_msg) != 0) {
      enqueue(msg);
      strcpy(car.last_status_msg, msg);
    }
  }
}

/* -------------------------------------------------------------- heartbeat */

static void observe_heartbeat(const struct timespec *now)
{
  if (car.shm->safety_heartbeat != car.last_heartbeat) {
    car.last_heartbeat = car.shm->safety_heartbeat;
    car.heartbeat_time = *now;
    car.heartbeat_seen = true;
  }
  /* Heartbeats seen during service or emergency mode do not count: after
     the mode is cleared the car waits for a new one before reconnecting. */
  if (in_mode()) {
    car.heartbeat_seen = false;
  }
}

/* Deadline for the next heartbeat change: every change gives 500 ms */
static void watchdog_deadline(struct timespec *t)
{
  *t = car.heartbeat_time;
  add_ms(t, WATCHDOG_MS);
}

static bool heartbeat_fresh(const struct timespec *now)
{
  struct timespec deadline;

  if (!car.heartbeat_seen) {
    return false;
  }
  watchdog_deadline(&deadline);
  return !time_reached(now, &deadline);
}

/* ------------------------------------------------------ the state machine */

static bool phase_is(const char *name)
{
  return strcmp(car.phase, name) == 0;
}

/* Changes the status and starts the timer for the new phase */
static void set_phase(const char *name)
{
  snprintf(car.shm->status, sizeof(car.shm->status), "%s", name);
  snprintf(car.phase, sizeof(car.phase), "%s", name);
  now_plus_ms(&car.phase_end, car.delay_ms);
}

/* Only these phases end by themselves. In service mode the doors stay open. */
static bool phase_has_end(void)
{
  if (phase_is("Open")) {
    return car.shm->service_mode != 1;
  }
  return phase_is("Opening") || phase_is("Closing") || phase_is("Between");
}

static void start_move(int target)
{
  int from = current_floor();

  set_target(target);
  if (next_floor(from, target > from ? 1 : -1, &car.moving_to)) {
    car.moving = true;
    set_phase("Between");
  }
}

/* Doors are closed: decide whether to move or open */
static void decide_when_closed(void)
{
  int current = current_floor();
  int target;

  if (car.shm->service_mode == 1) {
    /* Technician moves the car by setting target_floor. */
    if (!read_floor_field(car.shm->target_floor, &target) || !in_range(target)) {
      set_target(current);
    } else if (target != current) {
      start_move(target);
    }
  } else if (car.shm->emergency_mode == 1) {
    /* No new movement in emergency mode */
  } else if (car.has_floor_cmd) {
    car.has_floor_cmd = false;
    if (car.floor_cmd == current) {
      set_target(current);
      set_phase("Opening"); /* FLOOR for the current floor opens the doors */
    } else {
      start_move(car.floor_cmd);
    }
  } else if (!read_floor_field(car.shm->target_floor, &target) || !in_range(target)) {
    set_target(current);
  } else if (target != current) {
    start_move(target);
  }
}

/* One floor of travel has finished */
static void arrive(void)
{
  int target;

  car.moving = false;
  set_current(car.moving_to);

  if (in_mode()) {
    /* Stop at this floor: forget the target and pending commands.
       TODO (open question 2): in service mode a target more than one floor
       away is treated as "move one floor", then the target is reset. */
    set_target(car.moving_to);
    car.has_floor_cmd = false;
    set_phase("Closed");
    return;
  }

  if (car.has_floor_cmd) {
    car.has_floor_cmd = false;
    set_target(car.floor_cmd);
  }
  if (!read_floor_field(car.shm->target_floor, &target) || !in_range(target)) {
    target = car.moving_to;
    set_target(target);
  }
  if (target == car.moving_to) {
    set_phase("Opening");
  } else {
    start_move(target); /* keep going (or turn round) without stopping */
  }
}

/* The current phase's delay has passed */
static void finish_phase(void)
{
  if (phase_is("Opening")) {
    set_phase("Open");
  } else if (phase_is("Open")) {
    /* TODO (open question 1): doors also close automatically in emergency
       mode; only service mode keeps them open. */
    set_phase("Closing");
  } else if (phase_is("Closing")) {
    set_phase("Closed");
    decide_when_closed();
  } else if (phase_is("Between")) {
    arrive();
  }
}

/* One pass of the state machine. Called with the mutex held. */
static void step(const struct timespec *now)
{
  char status[9];
  int floor;

  /* Someone else changed the status (e.g. safety reopening the doors):
     that phase starts now. */
  read_status(status);
  if (strcmp(status, car.phase) != 0) {
    snprintf(car.phase, sizeof(car.phase), "%s", status);
    now_plus_ms(&car.phase_end, car.delay_ms);
    if (phase_is("Between") && !car.moving) {
      set_phase("Closed");
    } else if (!phase_is("Between")) {
      car.moving = false;
    }
  }

  /* Modes discard passenger and controller requests; invalid requests are
     discarded straight away. */
  if (in_mode()) {
    car.shm->floor_request_pending = 0;
    car.has_floor_cmd = false;
  } else if (car.shm->floor_request_pending == 1 && !valid_request(&floor)) {
    car.shm->floor_request_pending = 0;
  }

  if (car.shm->open_button == 1) {
    car.shm->open_button = 0;
    if (phase_is("Open")) {
      set_phase("Open"); /* restart the open timer */
    } else if (phase_is("Closing") || phase_is("Closed")) {
      set_phase("Opening");
    }
  }
  if (car.shm->close_button == 1) {
    car.shm->close_button = 0;
    if (phase_is("Open")) {
      set_phase("Closing");
    }
  }

  if (phase_is("Closed")) {
    decide_when_closed();
  } else if (phase_has_end() && time_reached(now, &car.phase_end)) {
    finish_phase();
  }
}

/* ----------------------------------------------------------- main thread */

static void run_car(void)
{
  static const char watchdog_message[] = "Safety system disconnected! Entering emergency mode.\n";
  car_shared_mem *shm = car.shm;
  char before[sizeof(car_shared_mem)];
  /* The fields from status to the end of the structure (everything except
     the mutex and condition variable), found with pointer arithmetic */
  char *fields = shm->status;
  size_t size = sizeof(car_shared_mem) - (size_t)(fields - (char *)shm);

  pthread_mutex_lock(&shm->mutex);
  while (!car.shutting_down) {
    struct timespec now;
    struct timespec watchdog;
    const struct timespec *wake = NULL;
    bool print_watchdog = false;

    clock_gettime(CLOCK_MONOTONIC, &now);
    memcpy(before, fields, size);

    observe_heartbeat(&now);
    if (car.connected && shm->emergency_mode == 0 && car.heartbeat_seen) {
      watchdog_deadline(&watchdog);
      if (time_reached(&now, &watchdog)) {
        shm->emergency_mode = 1;
        print_watchdog = true;
      }
    }

    step(&now);
    queue_status_if_changed();
    if (memcmp(before, fields, size) != 0) {
      pthread_cond_broadcast(&shm->cond);
    }

    if (print_watchdog) {
      /* write() is unbuffered, so the message is seen straight away */
      pthread_mutex_unlock(&shm->mutex);
      (void)write(STDOUT_FILENO, watchdog_message, sizeof(watchdog_message) - 1);
      pthread_mutex_lock(&shm->mutex);
      continue;
    }

    /* Sleep until the phase ends, the watchdog expires or anything changes */
    if (phase_has_end()) {
      wake = &car.phase_end;
    }
    if (car.connected && shm->emergency_mode == 0 && car.heartbeat_seen) {
      watchdog_deadline(&watchdog);
      wake = earlier(wake, &watchdog);
    }
    if (wake != NULL) {
      pthread_cond_timedwait(&shm->cond, &shm->mutex, wake);
    } else {
      pthread_cond_wait(&shm->cond, &shm->mutex);
    }
  }
  pthread_mutex_unlock(&shm->mutex);
}

/* -------------------------------------------------------- receiver thread */

static void *receive_commands(void *arg)
{
  int fd = *(int *)arg;
  char *msg;
  int floor;

  while ((msg = recv_msg(fd)) != NULL) {
    /* Only "FLOOR {floor}" with a floor in range is accepted; anything else
       is ignored. */
    if (strlen(msg) > 6 && memcmp(msg, "FLOOR ", 6) == 0 && floor_to_int(msg + 6, &floor) &&
        in_range(floor)) {
      pthread_mutex_lock(&car.shm->mutex);
      car.floor_cmd = floor;
      car.has_floor_cmd = true;
      pthread_cond_broadcast(&car.shm->cond);
      pthread_mutex_unlock(&car.shm->mutex);
    }
    free(msg);
  }

  pthread_mutex_lock(&car.shm->mutex);
  car.receiver_done = true;
  pthread_cond_broadcast(&car.shm->cond);
  pthread_mutex_unlock(&car.shm->mutex);
  return NULL;
}

/* --------------------------------------------------------- network thread */

/* Called with the mutex held */
static void start_connection(int fd)
{
  static int receiver_fd;
  char msg[MSG_SIZE];

  car.fd = fd;
  car.connected = true;
  car.receiver_done = false;
  car.mode_announced = false;
  car.queue_head = 0;
  car.queue_count = 0;

  snprintf(msg, sizeof(msg), "CAR %s %s %s", car.name, car.lowest_text, car.highest_text);
  enqueue(msg);
  format_status(car.last_status_msg);
  enqueue(car.last_status_msg);

  receiver_fd = fd;
  if (pthread_create(&receiver_thread, NULL, receive_commands, &receiver_fd) != 0) {
    car.receiver_done = true; /* treat as an immediate disconnection */
  }
  /* Wake the main thread so it starts timing the heartbeat watchdog */
  pthread_cond_broadcast(&car.shm->cond);
}

/* Called with the mutex held; releases it while waiting for the receiver */
static void drop_connection(void)
{
  int fd = car.fd;

  car.connected = false;
  car.fd = -1;
  car.queue_count = 0;
  shutdown(fd, SHUT_RDWR); /* wakes the receiver */
  pthread_mutex_unlock(&car.shm->mutex);
  pthread_join(receiver_thread, NULL);
  close(fd);
  pthread_mutex_lock(&car.shm->mutex);
}

/* Sends one message without holding the mutex. Called with it held. */
static bool send_unlocked(const char *msg)
{
  char copy[MSG_SIZE];
  int fd = car.fd;
  int result;

  snprintf(copy, sizeof(copy), "%s", msg);
  pthread_mutex_unlock(&car.shm->mutex);
  result = send_msg(fd, copy);
  pthread_mutex_lock(&car.shm->mutex);
  return result == 0;
}

/* One step while connected. Called with the mutex held. */
static void serve_connection(void)
{
  int floor;
  char msg[MSG_SIZE];

  if (car.receiver_done || car.shutting_down) {
    drop_connection();
  } else if (car.queue_count > 0) {
    snprintf(msg, sizeof(msg), "%s", car.queue[car.queue_head]);
    car.queue_head = (car.queue_head + 1) % QUEUE_SIZE;
    car.queue_count--;
    if (!send_unlocked(msg)) {
      drop_connection();
    }
  } else if (car.mode_announced) {
    drop_connection(); /* SERVICE or EMERGENCY has been sent */
  } else if (in_mode()) {
    enqueue(car.shm->service_mode == 1 ? "SERVICE" : "EMERGENCY");
    car.mode_announced = true;
  } else if (valid_request(&floor)) {
    char floor_text[4];
    int_to_floor(floor, floor_text);
    snprintf(msg, sizeof(msg), "SELECT %s", floor_text);
    if (!send_unlocked(msg)) {
      drop_connection();
    } else {
      /* Clear the mailbox only once the request has been sent */
      car.shm->floor_request_pending = 0;
      pthread_cond_broadcast(&car.shm->cond);
    }
  } else {
    pthread_cond_wait(&car.shm->cond, &car.shm->mutex);
  }
}

static void *network(void *unused)
{
  struct timespec next_attempt = {0, 0};
  struct timespec now;
  int fd;

  (void)unused;
  pthread_mutex_lock(&car.shm->mutex);
  while (!car.shutting_down) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    observe_heartbeat(&now);

    if (car.connected) {
      serve_connection();
    } else if (in_mode() || !heartbeat_fresh(&now)) {
      pthread_cond_wait(&car.shm->cond, &car.shm->mutex);
    } else if (!time_reached(&now, &next_attempt)) {
      pthread_cond_timedwait(&car.shm->cond, &car.shm->mutex, &next_attempt);
    } else {
      pthread_mutex_unlock(&car.shm->mutex);
      fd = connect_controller();
      pthread_mutex_lock(&car.shm->mutex);
      if (fd == -1) {
        now_plus_ms(&next_attempt, car.delay_ms);
      } else if (car.shutting_down) {
        close(fd);
      } else {
        start_connection(fd);
      }
    }
  }
  if (car.connected) {
    drop_connection();
  }
  pthread_mutex_unlock(&car.shm->mutex);
  return NULL;
}

/* ---------------------------------------------------------- signal thread */

static void *wait_for_sigint(void *arg)
{
  sigset_t *set = arg;
  int signal_number;

  if (sigwait(set, &signal_number) == 0) {
    pthread_mutex_lock(&car.shm->mutex);
    car.shutting_down = true;
    if (car.connected) {
      shutdown(car.fd, SHUT_RDWR); /* wakes a blocked send or receive */
    }
    pthread_cond_broadcast(&car.shm->cond);
    pthread_mutex_unlock(&car.shm->mutex);
  }
  return NULL;
}

/* ---------------------------------------------------------------- start-up */

/* Canonical whole number from 1 to 60000 */
static bool parse_delay(const char *text, long *delay)
{
  long value = 0;
  size_t len = strlen(text);

  if (len < 1 || len > 5 || text[0] < '1' || text[0] > '9') {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    if (text[i] < '0' || text[i] > '9') {
      return false;
    }
    value = value * 10 + (text[i] - '0');
  }
  if (value > 60000) {
    return false;
  }
  *delay = value;
  return true;
}

static bool parse_arguments(int argc, char *argv[])
{
  if (argc != 5) {
    printf("Usage: %s {name} {delay} {lowest floor} {highest floor}\n", argv[0]);
    return false;
  }
  if (!valid_car_name(argv[1])) {
    printf("Invalid car name.\n");
    return false;
  }
  if (!parse_delay(argv[2], &car.delay_ms)) {
    printf("Invalid delay specified.\n");
    return false;
  }
  if (!floor_to_int(argv[3], &car.lowest) || !floor_to_int(argv[4], &car.highest)) {
    printf("Invalid floor specified.\n");
    return false;
  }
  if (car.lowest >= car.highest) {
    printf("Invalid floor range: the lowest floor must be below the highest floor.\n");
    return false;
  }
  snprintf(car.name, sizeof(car.name), "%s", argv[1]);
  snprintf(car.lowest_text, sizeof(car.lowest_text), "%s", argv[3]);
  snprintf(car.highest_text, sizeof(car.highest_text), "%s", argv[4]);
  snprintf(car.shm_name, sizeof(car.shm_name), "%s%s", SHM_PREFIX, car.name);
  return true;
}

static bool create_shared_memory(void)
{
  pthread_mutexattr_t mutex_attr;
  pthread_condattr_t cond_attr;
  car_shared_mem *shm;
  int fd = shm_open(car.shm_name, O_CREAT | O_EXCL | O_RDWR, 0600);

  if (fd == -1) {
    perror("shm_open()");
    return false;
  }
  if (ftruncate(fd, sizeof(car_shared_mem)) == -1) {
    perror("ftruncate()");
    close(fd);
    shm_unlink(car.shm_name);
    return false;
  }
  shm = mmap(NULL, sizeof(car_shared_mem), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (shm == MAP_FAILED) {
    perror("mmap()");
    shm_unlink(car.shm_name);
    return false;
  }

  pthread_mutexattr_init(&mutex_attr);
  pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
  pthread_mutex_init(&shm->mutex, &mutex_attr);
  pthread_mutexattr_destroy(&mutex_attr);

  pthread_condattr_init(&cond_attr);
  pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED);
  pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC);
  pthread_cond_init(&shm->cond, &cond_attr);
  pthread_condattr_destroy(&cond_attr);

  /* Every other field is already 0 from ftruncate */
  strcpy(shm->status, "Closed");
  strcpy(shm->current_floor, car.lowest_text);
  strcpy(shm->target_floor, car.lowest_text);
  car.shm = shm;
  return true;
}

int main(int argc, char *argv[])
{
  static sigset_t sigint_set;
  pthread_t signal_thread;
  pthread_t network_thread;

  if (!parse_arguments(argc, argv)) {
    return 1;
  }

  /* Block SIGINT in every thread; the signal thread receives it with sigwait */
  sigemptyset(&sigint_set);
  sigaddset(&sigint_set, SIGINT);
  pthread_sigmask(SIG_BLOCK, &sigint_set, NULL);

  if (!create_shared_memory()) {
    return 1;
  }
  strcpy(car.phase, "Closed");
  car.fd = -1;

  if (pthread_create(&signal_thread, NULL, wait_for_sigint, &sigint_set) != 0 ||
      pthread_create(&network_thread, NULL, network, NULL) != 0) {
    fprintf(stderr, "Unable to start threads.\n");
    munmap(car.shm, sizeof(car_shared_mem));
    shm_unlink(car.shm_name);
    return 1;
  }

  run_car(); /* returns after SIGINT */

  pthread_join(network_thread, NULL);
  pthread_join(signal_thread, NULL);
  munmap(car.shm, sizeof(car_shared_mem));
  shm_unlink(car.shm_name);
  return 0;
}
