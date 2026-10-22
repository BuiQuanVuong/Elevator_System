/*
 * controller.c - Schedules all connected cars using collective control.
 *
 * Usage: ./controller
 *
 * Listens on 127.0.0.1:3001. Every connection gets its own thread:
 *   - a hall call pad sends one CALL and gets ACCEPTED or UNAVAILABLE
 *   - a car sends CAR, then STATUS / SELECT / SERVICE / EMERGENCY
 * The main thread accepts connections and joins the threads that have
 * finished; a signal thread waits for SIGINT with sigwait() and shuts
 * everything down.
 *
 * Locking:
 *   lock           - the car table (every field of every car_t, including
 *                    routes) and the connection table
 *   car->send_lock - sending FLOOR to that car. Held while send() runs so
 *                    FLOORs go out in order and the car's socket is not
 *                    closed mid-send; `lock` is never held during send().
 *   Order: a thread holding a send_lock may take `lock`, never the reverse.
 *
 * Each car's route is a list of stops. Every stop belongs to a sweep:
 * sweep 0 travels in first_dir, sweep 1 the opposite way, sweep 2 the same
 * way as sweep 0, and so on. Within a sweep the stops are in travel order.
 */

#include "shared.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define MAX_CONNECTIONS 1024
#define MAX_CARS 256
#define MAX_STOPS 4096
#define MAX_WORDS 5

typedef struct
{
  int floor;
  int sweep;
} stop_t;

typedef struct
{
  bool in_use;
  bool ready;       /* a valid STATUS has been received */
  int owner;        /* id of the connection that registered the car */
  int fd;
  char name[32];
  int lowest;
  int highest;
  int current;      /* from the latest STATUS */
  int target;
  char status[8];
  int commanded;    /* floor in the last FLOOR decided, 0 if none */
  bool must_send;   /* FLOOR commanded has not been sent yet */
  int first_dir;    /* direction of sweep 0: +1 up, -1 down */
  int stop_count;
  stop_t stops[MAX_STOPS];
  pthread_mutex_t send_lock; /* see "Locking" above */
} car_t;

typedef struct
{
  bool used;
  bool finished;
  int id;
  int fd;
  pthread_t thread;
} connection_t;

static pthread_mutex_t lock; /* see "Locking" above; initialised in main */
static car_t cars[MAX_CARS];
static connection_t connections[MAX_CONNECTIONS];
static int listener = -1;
static bool stopping = false;
static int next_id = 1;

/* ------------------------------------------------------------- messages */

/* Splits msg in place at single spaces. Returns the number of words, or -1
   if a word is empty (leading, trailing or double spaces) or there are too
   many words. */
static int split(char *msg, char *words[], int max)
{
  int count = 0;
  char *start = msg;

  for (char *p = msg;; p++) {
    if (*p == ' ' || *p == '\0') {
      bool end = (*p == '\0');
      if (p == start || count == max) {
        return -1;
      }
      *p = '\0';
      words[count++] = start;
      if (end) {
        return count;
      }
      start = p + 1;
    }
  }
}

/* True if the message's first word is `word` */
static bool first_word_is(const char *msg, const char *word)
{
  size_t len = strlen(word);

  return strlen(msg) >= len && memcmp(msg, word, len) == 0 &&
         (msg[len] == ' ' || msg[len] == '\0');
}

static bool valid_status_word(const char *word)
{
  return strcmp(word, "Opening") == 0 || strcmp(word, "Open") == 0 ||
         strcmp(word, "Closing") == 0 || strcmp(word, "Closed") == 0 ||
         strcmp(word, "Between") == 0;
}

/* --------------------------------------------------------------- cars */

static bool in_range(const car_t *car, int floor)
{
  return floor >= car->lowest && floor <= car->highest;
}

static car_t *find_car(const char *name)
{
  for (int i = 0; i < MAX_CARS; i++) {
    if (cars[i].in_use && strcmp(cars[i].name, name) == 0) {
      return &cars[i];
    }
  }
  return NULL;
}

/* A new registration replaces any car with the same name. Returns NULL if
   the table is full. Called with the lock held. */
static car_t *register_car(const char *name, int lowest, int highest, const connection_t *conn)
{
  car_t *car = find_car(name);

  for (int i = 0; car == NULL && i < MAX_CARS; i++) {
    if (!cars[i].in_use) {
      car = &cars[i];
    }
  }
  if (car != NULL) {
    car->in_use = true;
    car->ready = false;
    car->owner = conn->id;
    car->fd = conn->fd;
    snprintf(car->name, sizeof(car->name), "%s", name);
    car->lowest = lowest;
    car->highest = highest;
    car->current = lowest;
    car->target = lowest;
    car->status[0] = '\0';
    car->commanded = 0;
    car->must_send = false;
    car->first_dir = 1;
    car->stop_count = 0;
  }
  return car;
}

/* ----------------------------------------------------------- the route */

static int sign(int x)
{
  return (x > 0) - (x < 0);
}

static int sweep_dir(const car_t *car, int sweep)
{
  return (sweep % 2 == 0) ? car->first_dir : -car->first_dir;
}

/* Where new requests are measured from: the current floor, or the next
   floor in the direction of travel while the car is Closing or Between. */
static int position(const car_t *car)
{
  int dir = sign(car->target - car->current);
  int next;

  if (dir != 0 && (strcmp(car->status, "Closing") == 0 || strcmp(car->status, "Between") == 0) &&
      next_floor(car->current, dir, &next)) {
    return next;
  }
  return car->current;
}

static void insert_at(car_t *car, int index, int floor, int sweep)
{
  if (car->stop_count == MAX_STOPS) {
    return;
  }
  for (int i = car->stop_count; i > index; i--) {
    car->stops[i] = car->stops[i - 1];
  }
  car->stops[index].floor = floor;
  car->stops[index].sweep = sweep;
  car->stop_count++;
}

/* Adds the floor to a sweep in travel order; a floor already in that sweep
   is not added again. */
static void insert_in_sweep(car_t *car, int floor, int sweep)
{
  int dir = sweep_dir(car, sweep);
  int i = 0;

  while (i < car->stop_count && car->stops[i].sweep < sweep) {
    i++;
  }
  while (i < car->stop_count && car->stops[i].sweep == sweep) {
    if (car->stops[i].floor == floor) {
      return; /* duplicate */
    }
    if ((dir > 0) ? (car->stops[i].floor > floor) : (car->stops[i].floor < floor)) {
      break;
    }
    i++;
  }
  insert_at(car, i, floor, sweep);
}

/* kind: +1 for an UP call, -1 for a DOWN call, 0 for a passenger SELECT */
static void add_stop(car_t *car, int floor, int kind)
{
  int pos = position(car);
  int first;
  int toward;
  bool approaching;
  bool on_the_way;
  int sweep;

  if (car->stop_count == 0) {
    car->first_dir = (kind != 0) ? kind : ((floor >= pos) ? 1 : -1);
    insert_at(car, 0, floor, 0);
    return;
  }

  /* The car may be travelling to the start of a sweep that goes the other
     way (e.g. up to a DOWN call). A request in the direction of that
     journey, between the car and the first stop, is served on the way as a
     new sweep in front. */
  first = car->stops[0].floor;
  toward = sign(first - pos);
  approaching = toward != 0 && toward != car->first_dir;
  if (toward > 0) {
    on_the_way = floor >= pos && (kind == 0 ? floor < first : floor <= first);
  } else {
    on_the_way = floor <= pos && (kind == 0 ? floor > first : floor >= first);
  }
  if (approaching && (kind == toward || kind == 0) && on_the_way) {
    for (int i = 0; i < car->stop_count; i++) {
      car->stops[i].sweep++;
    }
    car->first_dir = toward;
    insert_at(car, 0, floor, 0);
    return;
  }

  /* Otherwise the earliest sweep in the right direction that does not
     need the car to reverse. Floors behind the car wait for a later sweep. */
  for (sweep = 0;; sweep++) {
    int dir = sweep_dir(car, sweep);
    if (kind != 0 && dir != kind) {
      continue;
    }
    if (sweep == 0 && !approaching && ((dir > 0) ? (floor < pos) : (floor > pos))) {
      continue;
    }
    break;
  }
  insert_in_sweep(car, floor, sweep);
}

static void remove_first_stop(car_t *car)
{
  int skipped;

  for (int i = 1; i < car->stop_count; i++) {
    car->stops[i - 1] = car->stops[i];
  }
  car->stop_count--;

  /* Renumber so the first remaining stop is in sweep 0 */
  if (car->stop_count > 0 && car->stops[0].sweep > 0) {
    skipped = car->stops[0].sweep;
    if (skipped % 2 == 1) {
      car->first_dir = -car->first_dir;
    }
    for (int i = 0; i < car->stop_count; i++) {
      car->stops[i].sweep -= skipped;
    }
  }
}

/* Decides to send FLOOR for the first stop. Called with `lock` held; the
   message itself is sent later by send_pending_floor() without `lock`. */
static void send_first_stop(car_t *car)
{
  car->commanded = car->stops[0].floor;
  car->must_send = true;
}

/* Sends the car's pending FLOOR, if any. Must be called WITHOUT `lock`.
   If several FLOORs were decided in the meantime only the latest is sent,
   since each one replaces the car's target. A failed send is noticed by
   the car's own connection thread. */
static void send_pending_floor(car_t *car, int owner)
{
  char floor[4];
  char msg[16];
  int fd = -1;

  pthread_mutex_lock(&car->send_lock);
  pthread_mutex_lock(&lock);
  if (car->in_use && car->owner == owner && car->must_send) {
    int_to_floor(car->commanded, floor);
    snprintf(msg, sizeof(msg), "FLOOR %s", floor);
    fd = car->fd;
    car->must_send = false;
  }
  pthread_mutex_unlock(&lock);
  if (fd != -1) {
    (void)send_msg(fd, msg);
  }
  pthread_mutex_unlock(&car->send_lock);
}

/* Adds a request and sends FLOOR if the first stop now needs commanding */
static void request(car_t *car, int floor, int kind)
{
  bool had_first = car->stop_count > 0;
  int old_floor = had_first ? car->stops[0].floor : 0;
  int old_dir = had_first ? sweep_dir(car, 0) : 0;
  bool changed;
  bool closed_here;

  add_stop(car, floor, kind);

  floor = car->stops[0].floor;
  changed = !had_first || floor != old_floor || sweep_dir(car, 0) != old_dir;
  closed_here = strcmp(car->status, "Closed") == 0 && car->current == floor;
  if (floor != car->commanded || (changed && closed_here)) {
    send_first_stop(car);
  }
}

/* ------------------------------------------------------- hall call pads */

static void handle_call(int fd, char *msg)
{
  char *words[MAX_WORDS];
  int count = split(msg, words, MAX_WORDS);
  const char *reply = "UNAVAILABLE";
  int floor;
  int dir;
  car_t *car = NULL;
  int owner = 0;

  if (count == 4 && strcmp(words[0], "CALL") == 0 && valid_car_name(words[1]) &&
      floor_to_int(words[2], &floor) &&
      (strcmp(words[3], "UP") == 0 || strcmp(words[3], "DOWN") == 0)) {
    dir = (strcmp(words[3], "UP") == 0) ? 1 : -1;
    pthread_mutex_lock(&lock);
    car = find_car(words[1]);
    if (car != NULL && car->ready && in_range(car, floor) &&
        !(dir > 0 && floor == car->highest) && !(dir < 0 && floor == car->lowest)) {
      request(car, floor, dir);
      owner = car->owner;
      reply = "ACCEPTED";
    }
    pthread_mutex_unlock(&lock);
    if (owner != 0) {
      send_pending_floor(car, owner);
    }
  }
  (void)send_msg(fd, reply);
}

/* ------------------------------------------------------------------ cars */

/* Handles one message from a registered car. Returns false if the car must
   be removed (SERVICE, EMERGENCY or an invalid message). Lock held. */
static bool car_message(car_t *car, char *msg)
{
  char *words[MAX_WORDS];
  int count = split(msg, words, MAX_WORDS);
  int current;
  int target;
  int floor;

  if (count == 4 && strcmp(words[0], "STATUS") == 0 && floor_to_int(words[1], &current) &&
      floor_to_int(words[2], &target) && in_range(car, current) && in_range(car, target) &&
      valid_status_word(words[3])) {
    car->current = current;
    car->target = target;
    snprintf(car->status, sizeof(car->status), "%s", words[3]);
    car->ready = true;
    /* Arrived at the first stop: remove it and send the next one */
    if (strcmp(car->status, "Opening") == 0 && car->stop_count > 0 &&
        car->stops[0].floor == current) {
      remove_first_stop(car);
      if (car->stop_count > 0) {
        send_first_stop(car);
      }
    }
    return true;
  }
  if (car->ready && count == 2 && strcmp(words[0], "SELECT") == 0 &&
      floor_to_int(words[1], &floor) && in_range(car, floor)) {
    request(car, floor, 0);
    return true;
  }
  /* SERVICE, EMERGENCY, anything before the first STATUS, or invalid */
  return false;
}

static void serve_car(connection_t *conn, char *registration)
{
  char *words[MAX_WORDS];
  int count = split(registration, words, MAX_WORDS);
  int lowest;
  int highest;
  car_t *car;
  char *msg;
  bool keep = true;

  if (count != 4 || strcmp(words[0], "CAR") != 0 || !valid_car_name(words[1]) ||
      !floor_to_int(words[2], &lowest) || !floor_to_int(words[3], &highest) ||
      lowest >= highest) {
    return; /* invalid registration: close the connection */
  }

  pthread_mutex_lock(&lock);
  car = register_car(words[1], lowest, highest, conn);
  pthread_mutex_unlock(&lock);
  if (car == NULL) {
    return;
  }

  while (keep && (msg = recv_msg(conn->fd)) != NULL) {
    pthread_mutex_lock(&lock);
    if (car->in_use && car->owner == conn->id) {
      keep = car_message(car, msg);
    } else {
      keep = false; /* replaced by a newer registration */
    }
    pthread_mutex_unlock(&lock);
    free(msg);
    if (keep) {
      send_pending_floor(car, conn->id);
    }
  }

  /* The car and its whole route go, unless a newer connection owns it.
     Taking send_lock first means no other thread is sending on this
     connection when the caller closes it. */
  pthread_mutex_lock(&car->send_lock);
  pthread_mutex_lock(&lock);
  if (car->in_use && car->owner == conn->id) {
    car->in_use = false;
  }
  pthread_mutex_unlock(&lock);
  pthread_mutex_unlock(&car->send_lock);
}

/* ----------------------------------------------------------- connections */

static void *serve_connection(void *arg)
{
  connection_t *conn = arg;
  char *msg = recv_msg(conn->fd);

  if (msg != NULL) {
    if (first_word_is(msg, "CALL")) {
      handle_call(conn->fd, msg);
    } else if (first_word_is(msg, "CAR")) {
      serve_car(conn, msg);
    }
    free(msg);
  }

  pthread_mutex_lock(&lock);
  close(conn->fd);
  conn->fd = -1;
  conn->finished = true;
  pthread_mutex_unlock(&lock);
  return NULL;
}

/* Joins threads that have finished so their slots can be reused. Lock held. */
static void reap_finished(void)
{
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].used && connections[i].finished) {
      pthread_join(connections[i].thread, NULL);
      connections[i].used = false;
    }
  }
}

static void start_connection(int fd)
{
  connection_t *conn = NULL;

  for (int i = 0; conn == NULL && i < MAX_CONNECTIONS; i++) {
    if (!connections[i].used) {
      conn = &connections[i];
    }
  }
  if (conn == NULL) {
    close(fd); /* too many connections */
    return;
  }
  conn->used = true;
  conn->finished = false;
  conn->id = next_id++;
  conn->fd = fd;
  if (pthread_create(&conn->thread, NULL, serve_connection, conn) != 0) {
    close(fd);
    conn->used = false;
  }
}

static void *wait_for_sigint(void *arg)
{
  sigset_t *set = arg;
  int signal_number;

  if (sigwait(set, &signal_number) == 0) {
    pthread_mutex_lock(&lock);
    stopping = true;
    shutdown(listener, SHUT_RDWR); /* wakes accept() */
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
      if (connections[i].used && !connections[i].finished) {
        shutdown(connections[i].fd, SHUT_RDWR); /* wakes that thread */
      }
    }
    pthread_mutex_unlock(&lock);
  }
  return NULL;
}

static int open_listener(void)
{
  struct sockaddr_in addr;
  int enable = 1;
  int fd = socket(AF_INET, SOCK_STREAM, 0);

  if (fd == -1) {
    perror("socket()");
    return -1;
  }
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) == -1) {
    perror("setsockopt()");
    close(fd);
    return -1;
  }
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(CONTROLLER_PORT);
  if (inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) != 1 ||
      bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
    perror("bind()");
    close(fd);
    return -1;
  }
  if (listen(fd, 128) == -1) {
    perror("listen()");
    close(fd);
    return -1;
  }
  return fd;
}

int main(void)
{
  static sigset_t sigint_set;
  pthread_t signal_thread;
  int fd;

  /* Block SIGINT in every thread; the signal thread receives it with sigwait */
  sigemptyset(&sigint_set);
  sigaddset(&sigint_set, SIGINT);
  pthread_sigmask(SIG_BLOCK, &sigint_set, NULL);
  pthread_mutex_init(&lock, NULL);
  for (int i = 0; i < MAX_CARS; i++) {
    pthread_mutex_init(&cars[i].send_lock, NULL);
  }

  listener = open_listener();
  if (listener == -1) {
    return 1;
  }
  if (pthread_create(&signal_thread, NULL, wait_for_sigint, &sigint_set) != 0) {
    fprintf(stderr, "Unable to start the signal thread.\n");
    close(listener);
    return 1;
  }

  for (;;) {
    fd = accept(listener, NULL, NULL);
    pthread_mutex_lock(&lock);
    if (stopping) {
      if (fd != -1) {
        close(fd);
      }
      pthread_mutex_unlock(&lock);
      break;
    }
    if (fd != -1) {
      reap_finished();
      start_connection(fd);
    }
    pthread_mutex_unlock(&lock);
  }

  /* Every connection has been shut down; wait for the threads to finish */
  for (int i = 0; i < MAX_CONNECTIONS; i++) {
    if (connections[i].used) {
      pthread_join(connections[i].thread, NULL);
    }
  }
  pthread_join(signal_thread, NULL);
  close(listener);
  return 0;
}
