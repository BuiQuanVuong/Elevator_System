#ifndef COMMON_H
#define COMMON_H

#include <pthread.h>
#include <stdint.h>

#define CONTROLLER_PORT 3001
#define SHM_PREFIX "/cab403_car_"
#define MAX_MSG_LEN 4096

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

/* Floors are stored as ints: B99..B1 -> -99..-1, 1..999 -> 1..999 (no 0) */
int floor_to_int(const char *s, int *out);   /* 1 if s is a canonical floor */
void int_to_floor(int floor, char buf[4]);
int next_floor(int floor, int dir, int *out); /* dir +1 up / -1 down; 1 if it exists */

int valid_car_name(const char *s);

int send_msg(int fd, const char *msg);        /* 0 on success, -1 on error */
char *recv_msg(int fd);                       /* malloc'd string, NULL on error/disconnect */
int connect_controller(void);                 /* socket fd, or -1 */

#endif
