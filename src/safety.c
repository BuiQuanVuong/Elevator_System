/*
 * safety.c - Safety system for one elevator car (SAFETY-CRITICAL)
 * Usage: ./safety {car name}
 *
 * Opens /cab403_car_{name} (never creates or unlinks it), increments
 * safety_heartbeat every 100 ms and, on every change, applies in order:
 *   1. obstruction while Closing          -> Opening
 *   2. emergency_stop, not in emergency   -> report, emergency mode, reset
 *   3. overload, not in emergency         -> report, emergency mode
 *   4. inconsistent data, not in emergency -> report, emergency mode
 *
 * SAFETY-CRITICAL PRACTICE (MISRA C:2012)
 *   - No dynamic memory or recursion (Dir 4.12, Rules 21.3, 17.2).
 *   - All shared memory is validated before use; strings are never assumed
 *     to be NUL-terminated (Dir 4.14).
 *   - Every return value that can signal failure is checked (Rule 17.7).
 *   - Fixed-width types, U suffixes, braces on every body, boolean
 *     conditions, no assignments in conditions, final else, single exit
 *     (Rules 14.4, 13.4, 15.6, 15.7, 15.5).
 *   - Narrowest scope; internal linkage except main (Rules 8.7, 8.9).
 *
 * DEVIATIONS
 * D1  Dir 1.1 - POSIX calls (shm_open, mmap, write, sigaction,
 *     clock_gettime, process-shared mutex/condvar) are required by the
 *     specification. Every result is checked.
 * D2  Rule 21.5 - <signal.h>, needed for a clean SIGINT exit. The handler
 *     only sets a volatile sig_atomic_t; the loop checks it every 100 ms.
 * D3  Rule 21.10 - <time.h>, needed for CLOCK_MONOTONIC deadlines. No
 *     calendar functions are used.
 * D4  Rule 11.5 - mmap's void * is cast to car_shared_mem *. mmap memory is
 *     page-aligned; the car sizes the object with ftruncate.
 * D5  Rule 8.9 - stop_requested has file scope, as the handler and main
 *     loop share it.
 * D6  No <stdio.h> (Rule 21.6): write() is unbuffered, so messages appear
 *     immediately. A failed write is ignored, as the safety action has
 *     already been taken.
 * D7  safety_heartbeat wraps from UINT32_MAX to 0 on purpose; readers only
 *     compare it for inequality.
 * D8  The mutex is not robust (its attributes are fixed by the spec). If
 *     another process dies holding it, this process blocks and the car's
 *     watchdog enters emergency mode.
 * D9  Other processes may change fields at any time, so every check and
 *     correction is made with the mutex held and repeated after each wake.
 * D10 Rule 22.10 - errno is tested after write(), which POSIX defines as
 *     errno-setting. EINTR is retried so SIGINT doesn't cut a message.
 *
 * Checked with: cppcheck --addon=misra --library=posix --enable=all safety.c
 * It reports only Rules 11.5, 21.5, 21.10 and 22.10 (above), plus a
 * checker config note that EINTR is undefined.
 */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define SHM_PREFIX "/cab403_car_"
#define SHM_PREFIX_LEN 12U                 /* strlen(SHM_PREFIX) */
#define MAX_NAME_LEN 31U
#define SHM_NAME_SIZE (SHM_PREFIX_LEN + MAX_NAME_LEN + 1U)

#define STATUS_SIZE 8U
#define FLOOR_SIZE 4U

#define NS_PER_SECOND 1000000000L
#define HEARTBEAT_PERIOD_NS 100000000L     /* 100 ms */

#define EXIT_OK 0
#define EXIT_FAILED 1

/* Exact layout from the specification - do not change */
typedef struct
{
  pthread_mutex_t mutex;          // Locked while the contents of the structure are being accessed/modified
  pthread_cond_t cond;            // Signalled when the contents of the structure change
  char status[8];                 // NUL-terminated C string indicating the elevator's status
  char current_floor[4];          // NUL-terminated C string in the range of "B99" to "B1" and "1" to "999"
  char target_floor[4];           // Controller's immediate movement target
  char floor_request[4];          // Passenger floor selection mailbox
  uint8_t open_button;            // 1 if open doors button is pressed, 0 otherwise
  uint8_t close_button;           // 1 if close doors button is pressed, 0 otherwise
  uint8_t floor_request_pending;  // 1 while floor_request awaits transmission
  uint8_t door_obstruction;       // 1 if obstruction detected, 0 otherwise
  uint8_t overload;               // 1 if overload detected
  uint8_t emergency_stop;         // 1 if emergency stop button has been pressed, 0 otherwise
  uint8_t service_mode;           // 1 while in individual service mode
  uint8_t emergency_mode;         // 0 if not in emergency mode, 1 if in emergency mode
  uint32_t safety_heartbeat;      // Sequence incremented by the safety process
} car_shared_mem;

typedef enum
{
  REPORT_NONE,
  REPORT_STOP,
  REPORT_OVERLOAD,
  REPORT_DATA_ERROR
} report_t;

/* D2, D5 */
static volatile sig_atomic_t stop_requested = 0;

static void on_sigint(int signal_number)
{
  (void)signal_number;
  stop_requested = 1;
}

/* OUTPUT */

/* Retries partial and interrupted writes (D6, D10) */
static void write_text(const char *text)
{
  const char *position = text;
  size_t remaining = strlen(text);
  bool failed = false;

  while ((remaining > 0U) && (!failed)) {
    ssize_t written = write(STDOUT_FILENO, position, remaining);
    if (written > 0) {
      position = &position[written];
      remaining -= (size_t)written;
    } else if ((written < 0) && (errno == EINTR)) {
      /* retry */
    } else {
      failed = true;
    }
  }
}

static void print_report(report_t report)
{
  switch (report) {
  case REPORT_STOP:
    write_text("The emergency stop button has been pressed!\n");
    break;
  case REPORT_OVERLOAD:
    write_text("The overload sensor has been tripped!\n");
    break;
  case REPORT_DATA_ERROR:
    write_text("Data consistency error!\n");
    break;
  default:
    /* REPORT_NONE */
    break;
  }
}

/* VALIDATION */

/* Returns size if the field has no NUL */
static size_t bounded_length(const char *field, size_t size)
{
  size_t length = 0U;

  while ((length < size) && (field[length] != '\0')) {
    length++;
  }
  return length;
}

static bool is_digit(char c)
{
  return (c >= '0') && (c <= '9');
}

static bool is_alphanumeric(char c)
{
  return ((c >= 'a') && (c <= 'z')) || ((c >= 'A') && (c <= 'Z')) || is_digit(c);
}

/* 1-31 letters, digits, '_' or '-', starting with a letter or digit */
static bool valid_car_name(const char *name, size_t *length)
{
  size_t count = 0U;
  bool valid = true;

  while (valid && (count <= MAX_NAME_LEN) && (name[count] != '\0')) {
    char c = name[count];
    if (is_alphanumeric(c) || ((count > 0U) && ((c == '_') || (c == '-')))) {
      count++;
    } else {
      valid = false;
    }
  }
  if ((count == 0U) || (count > MAX_NAME_LEN)) {
    valid = false;
  }
  *length = count;
  return valid;
}

/* "B1".."B99" or "1".."999", no leading zeroes, NUL within the field */
static bool valid_floor(const char field[FLOOR_SIZE])
{
  size_t length = bounded_length(field, FLOOR_SIZE);
  bool valid = true;

  if ((length == 0U) || (length >= FLOOR_SIZE)) {
    valid = false;
  } else {
    size_t first_digit = 0U;
    if (field[0] == 'B') {
      first_digit = 1U;
    }
    if (first_digit >= length) {
      valid = false; /* "B" alone */
    } else if ((field[first_digit] < '1') || (field[first_digit] > '9')) {
      valid = false; /* no number, zero or leading zero */
    } else {
      for (size_t i = first_digit + 1U; i < length; i++) {
        if (!is_digit(field[i])) {
          valid = false;
        }
      }
    }
  }
  return valid;
}

static bool status_is(const char field[STATUS_SIZE], const char *word)
{
  return (bounded_length(field, STATUS_SIZE) < STATUS_SIZE) && (strcmp(field, word) == 0);
}

static bool valid_status(const char field[STATUS_SIZE])
{
  return status_is(field, "Opening") || status_is(field, "Open") ||
         status_is(field, "Closing") || status_is(field, "Closed") ||
         status_is(field, "Between");
}

static bool data_is_consistent(const car_shared_mem *car)
{
  bool consistent = valid_floor(car->current_floor) && valid_floor(car->target_floor) &&
                    valid_status(car->status);

  if (car->floor_request_pending == 1U) {
    consistent = consistent && valid_floor(car->floor_request);
  }
  consistent = consistent && (car->open_button <= 1U) && (car->close_button <= 1U) &&
               (car->floor_request_pending <= 1U) && (car->door_obstruction <= 1U) &&
               (car->overload <= 1U) && (car->emergency_stop <= 1U) &&
               (car->service_mode <= 1U) && (car->emergency_mode <= 1U);
  if (car->door_obstruction == 1U) {
    consistent = consistent &&
                 (status_is(car->status, "Opening") || status_is(car->status, "Closing"));
  }
  return consistent;
}

/* RULES */

/* Mutex held. Only emergency_mode == 1 counts as emergency mode, so any
   other value is caught by rule 4. */
static report_t apply_rules(car_shared_mem *car, bool *changed)
{
  report_t report = REPORT_NONE;

  if ((car->door_obstruction == 1U) && status_is(car->status, "Closing")) {
    (void)memcpy(car->status, "Opening", sizeof("Opening"));
    *changed = true;
  }

  if ((car->emergency_stop == 1U) && (car->emergency_mode != 1U)) {
    car->emergency_mode = 1U;
    car->emergency_stop = 0U;
    report = REPORT_STOP;
    *changed = true;
  } else if ((car->overload == 1U) && (car->emergency_mode != 1U)) {
    car->emergency_mode = 1U;
    report = REPORT_OVERLOAD;
    *changed = true;
  } else if ((car->emergency_mode != 1U) && (!data_is_consistent(car))) {
    car->emergency_mode = 1U;
    report = REPORT_DATA_ERROR;
    *changed = true;
  } else {
    /* nothing to do */
  }
  return report;
}

/* TIMING */

static void add_nanoseconds(struct timespec *time, long nanoseconds)
{
  time->tv_nsec += nanoseconds;
  while (time->tv_nsec >= NS_PER_SECOND) {
    time->tv_nsec -= NS_PER_SECOND;
    time->tv_sec++;
  }
}

static bool time_reached(const struct timespec *now, const struct timespec *deadline)
{
  return (now->tv_sec > deadline->tv_sec) ||
         ((now->tv_sec == deadline->tv_sec) && (now->tv_nsec >= deadline->tv_nsec));
}

/* MAIN LOOP */

/* Runs until SIGINT; false if a clock or pthread call fails */
static bool run_safety(car_shared_mem *car)
{
  struct timespec now;
  struct timespec deadline;
  bool ok = (clock_gettime(CLOCK_MONOTONIC, &deadline) == 0);
  bool locked = false;

  if (ok) {
    add_nanoseconds(&deadline, HEARTBEAT_PERIOD_NS);
    locked = (pthread_mutex_lock(&car->mutex) == 0);
    ok = locked;
  }

  while (ok && (stop_requested == 0)) {
    bool changed = false;
    report_t report = REPORT_NONE;

    ok = (clock_gettime(CLOCK_MONOTONIC, &now) == 0);
    if (ok) {
      /* Fixed schedule: other wake-ups don't delay it */
      if (time_reached(&now, &deadline)) {
        car->safety_heartbeat++; /* D7 */
        changed = true;
        add_nanoseconds(&deadline, HEARTBEAT_PERIOD_NS);
        if (time_reached(&now, &deadline)) {
          /* fell behind: restart rather than burst */
          deadline = now;
          add_nanoseconds(&deadline, HEARTBEAT_PERIOD_NS);
        }
      }
      report = apply_rules(car, &changed);
      if (changed) {
        ok = (pthread_cond_broadcast(&car->cond) == 0);
      }
    }

    if (ok && (report != REPORT_NONE)) {
      /* Print unlocked, then re-check before waiting */
      ok = (pthread_mutex_unlock(&car->mutex) == 0);
      if (ok) {
        locked = false;
      }
      print_report(report);
      if (ok) {
        locked = (pthread_mutex_lock(&car->mutex) == 0);
        ok = locked;
      }
    } else if (ok && (stop_requested == 0)) {
      int result = pthread_cond_timedwait(&car->cond, &car->mutex, &deadline);
      ok = (result == 0) || (result == ETIMEDOUT);
    } else {
      /* stopping or failed */
    }
  }

  if (locked && (pthread_mutex_unlock(&car->mutex) != 0)) {
    ok = false;
  }
  return ok;
}

/* SETUP */

static bool install_sigint_handler(void)
{
  struct sigaction action;

  (void)memset(&action, 0, sizeof(action));
  action.sa_handler = on_sigint;
  action.sa_flags = 0;
  return (sigemptyset(&action.sa_mask) == 0) && (sigaction(SIGINT, &action, NULL) == 0);
}

/* NULL if the object doesn't exist or can't be mapped */
static car_shared_mem *open_car(const char *shm_name, int *fd)
{
  car_shared_mem *car = NULL;

  *fd = shm_open(shm_name, O_RDWR, 0);
  if (*fd >= 0) {
    void *mapping = mmap(NULL, sizeof(car_shared_mem), PROT_READ | PROT_WRITE, MAP_SHARED, *fd, 0);
    if (mapping != MAP_FAILED) {
      car = (car_shared_mem *)mapping; /* D4 */
    }
    if (car == NULL) {
      (void)close(*fd);
      *fd = -1;
    }
  }
  return car;
}

int main(int argc, char *argv[])
{
  int status = EXIT_FAILED;
  size_t name_length = 0U;

  if (argc != 2) {
    write_text("Usage: ./safety {car name}\n");
  } else if (!valid_car_name(argv[1], &name_length)) {
    write_text("Invalid car name.\n");
  } else if (!install_sigint_handler()) {
    write_text("Unable to install the SIGINT handler.\n");
  } else {
    char shm_name[SHM_NAME_SIZE];
    int fd = -1;
    car_shared_mem *car;

    (void)memcpy(shm_name, SHM_PREFIX, SHM_PREFIX_LEN);
    (void)memcpy(&shm_name[SHM_PREFIX_LEN], argv[1], name_length);
    shm_name[SHM_PREFIX_LEN + name_length] = '\0';

    car = open_car(shm_name, &fd);
    if (car == NULL) {
      write_text("Unable to access car ");
      write_text(argv[1]);
      write_text(".\n");
    } else {
      if (run_safety(car)) {
        status = EXIT_OK;
      } else {
        write_text("Safety system failure: unable to use the car's shared memory.\n");
      }
      if (munmap(car, sizeof(*car)) != 0) {
        status = EXIT_FAILED;
      }
      if (close(fd) != 0) {
        status = EXIT_FAILED;
      }
    }
  }
  return status;
}
