/*
 * safety.c - Safety system for one elevator car (SAFETY-CRITICAL COMPONENT)
 *
 * Usage: ./safety {car name}
 *
 * Opens the car's shared memory object /cab403_car_{car name} (it never
 * creates or unlinks it), increments safety_heartbeat every 100 ms and,
 * whenever the shared state changes, applies these rules in order:
 *   1. door_obstruction is 1 while status is Closing -> status = Opening
 *   2. emergency_stop is 1 and not in emergency mode -> report, enter
 *      emergency mode, reset emergency_stop
 *   3. overload is 1 and not in emergency mode       -> report, enter
 *      emergency mode
 *   4. not in emergency mode and the data is inconsistent -> report
 *      "Data consistency error!", enter emergency mode
 *
 * ---------------------------------------------------------------------------
 * SAFETY-CRITICAL PRACTICE
 * This file follows MISRA C:2012 as far as the required functionality
 * allows. In particular:
 *   - No dynamic memory allocation (Dir 4.12, Rule 21.3) and no recursion
 *     (Rule 17.2).
 *   - Every value read from shared memory is external data and is fully
 *     validated before use (Dir 4.14). Strings are never assumed to be
 *     NUL-terminated: every length is bounded by the size of its field.
 *   - The return value of every system and library call that can fail is
 *     checked (Rule 17.7, Dir 4.7).
 *   - Fixed-width types, unsigned constants with a U suffix, braces on every
 *     body, boolean controlling expressions, no assignments inside
 *     conditions, every if/else-if chain ends in else, and every function
 *     has a single point of exit (Rules 14.4, 13.4, 15.6, 15.7, 15.5).
 *   - Objects are declared at the narrowest scope possible; everything
 *     except main has internal linkage (Rules 8.7, 8.9).
 *
 * DOCUMENTED DEVIATIONS
 * D1  Dir 1.1 / Rule 1.1 - Use of POSIX interfaces that are not part of
 *     ISO C: shm_open, mmap, munmap, close, write, sigaction,
 *     clock_gettime and process-shared pthread mutex/condition variables.
 *     Justification: the specification requires POSIX shared memory and
 *     process-shared synchronisation; there is no ISO C alternative. Each
 *     call's result is checked and failure leads to a reported, orderly exit.
 * D2  Rule 21.5 - <signal.h> is used.
 *     Justification: the component must end cleanly on SIGINT (Ctrl+C) so
 *     that it never exits while holding the shared mutex. The handler only
 *     stores to a volatile sig_atomic_t, which is async-signal-safe. The
 *     main loop wakes at least every heartbeat period (100 ms) to see it.
 * D3  Rule 21.10 - <time.h> is used.
 *     Justification: clock_gettime(CLOCK_MONOTONIC) is needed for absolute
 *     heartbeat deadlines that are not affected by wall-clock changes or by
 *     unrelated condition-variable wake-ups. No calendar/date functions are
 *     used.
 * D4  Rule 11.5 - The void pointer returned by mmap is converted to a
 *     pointer to car_shared_mem.
 *     Justification: this is the only way to access a mapped object. mmap
 *     returns page-aligned memory, so the alignment is correct for the
 *     structure. The object's size is not checked separately (that would
 *     need fstat, which is outside the techniques used in this unit); the
 *     car creates the object with ftruncate to exactly this size.
 * D5  Rule 8.9 / Rule 8.4 - stop_requested has file scope.
 *     Justification: it is shared between the signal handler and the main
 *     loop, so it cannot be a block-scope object. It has internal linkage.
 * D6  Rule 21.6 is followed by NOT using <stdio.h>: output goes through
 *     write() (covered by D1), which is unbuffered, so required diagnostic
 *     messages reach the reader immediately without fflush. If writing a
 *     message fails, the safety action has already been taken; the failure
 *     is not otherwise reportable and is deliberately ignored.
 * D7  Rule 10.x (essential type model) / unsigned wraparound -
 *     safety_heartbeat is a uint32_t sequence counter that is expected to
 *     wrap from UINT32_MAX to 0. Unsigned wraparound is well defined and is
 *     intentional here; readers compare it for inequality only.
 * D8  Shared mutex robustness - The mutex is created by the car with the
 *     attributes fixed by the specification (not PTHREAD_MUTEX_ROBUST). If
 *     another process dies while holding it, this component blocks in
 *     pthread_mutex_lock. This cannot be corrected here without changing
 *     the car's initialisation, which the specification fixes. The car's
 *     own heartbeat watchdog detects the resulting loss of heartbeat and
 *     enters emergency mode, which is the intended fail-safe behaviour.
 * D9  Other processes may change shared fields between this component's
 *     checks; all checks and corrections are therefore made while holding
 *     the mutex, and the state is re-checked after every wake-up.
 * ---------------------------------------------------------------------------
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

/* Set by the SIGINT handler, read by the main loop (see D2, D5) */
static volatile sig_atomic_t stop_requested = 0;

static void on_sigint(int signal_number)
{
  (void)signal_number;
  stop_requested = 1;
}

/* ------------------------------------------------------------------ output */

/* Writes a NUL-terminated string to standard output, retrying partial
   writes and interrupted calls. A failed write cannot be reported (D6). */
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
      /* interrupted before anything was written: try again */
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
    /* REPORT_NONE: nothing to print */
    break;
  }
}

/* ---------------------------------------------------------------- validation */

/* Length of a string stored in a fixed-size field; returns size if the
   field contains no NUL terminator. */
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

/* Car name: 1-31 letters, digits, underscores or hyphens; the first
   character must be a letter or digit. Sets *length on success. */
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

/* Canonical floor: "B1".."B99" or "1".."999", NUL-terminated within the
   4-byte field, no leading zeroes, signs, spaces or trailing characters. */
static bool valid_floor(const char field[FLOOR_SIZE])
{
  size_t length = bounded_length(field, FLOOR_SIZE);
  size_t first_digit = 0U;
  bool valid = true;

  if ((length == 0U) || (length >= FLOOR_SIZE)) {
    valid = false;
  } else {
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

/* True if the status field holds exactly the given word */
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

/* --------------------------------------------------------------- the rules */

/* Applies the four rules in order. The caller must hold the mutex.
   Only an emergency_mode of exactly 1 counts as being in emergency mode, so
   any other value is caught as a data consistency error by rule 4.
   Sets *changed if shared memory was modified. */
static report_t apply_rules(car_shared_mem *car, bool *changed)
{
  report_t report = REPORT_NONE;

  if ((car->door_obstruction == 1U) && status_is(car->status, "Closing")) {
    (void)memcpy(car->status, "Opening", sizeof("Opening")); /* 8 bytes incl. NUL */
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
    /* already in emergency mode, or nothing wrong */
  }
  return report;
}

/* ------------------------------------------------------------------- timing */

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

/* -------------------------------------------------------------- main loop */

/* Runs until SIGINT. Holds the mutex while checking and changing shared
   memory, releases it while waiting and while printing. Returns false if a
   clock or synchronisation call fails. */
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
      /* Heartbeat on a fixed absolute schedule: wake-ups never restart it */
      if (time_reached(&now, &deadline)) {
        car->safety_heartbeat++; /* wraps intentionally (D7) */
        changed = true;
        add_nanoseconds(&deadline, HEARTBEAT_PERIOD_NS);
        if (time_reached(&now, &deadline)) {
          /* fell more than a period behind (e.g. mutex held by another
             process): restart the schedule instead of bursting */
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
      /* Print without holding the mutex, then re-check before waiting */
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
      /* stopping, or an error occurred */
    }
  }

  if (locked && (pthread_mutex_unlock(&car->mutex) != 0)) {
    ok = false;
  }
  return ok;
}

/* ------------------------------------------------------------------- setup */

static bool install_sigint_handler(void)
{
  struct sigaction action;

  (void)memset(&action, 0, sizeof(action));
  action.sa_handler = on_sigint;
  action.sa_flags = 0;
  return (sigemptyset(&action.sa_mask) == 0) && (sigaction(SIGINT, &action, NULL) == 0);
}

/* Opens and maps an existing car object. Returns NULL if it does not exist
   or cannot be mapped. */
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
      if (munmap(car, sizeof(car_shared_mem)) != 0) {
        status = EXIT_FAILED;
      }
      if (close(fd) != 0) {
        status = EXIT_FAILED;
      }
    }
  }
  return status;
}
