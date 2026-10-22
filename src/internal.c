#include "common.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* Applies one operation while the mutex is held.
   Returns 1 if shared memory changed, 0 if not (an error was printed). */
static int apply(car_shared_mem *shm, const char *op, const char *floor_arg)
{
  int current;
  int target;

  if (strcmp(op, "open") == 0) {
    shm->open_button = 1;
  } else if (strcmp(op, "close") == 0) {
    shm->close_button = 1;
  } else if (strcmp(op, "stop") == 0) {
    shm->emergency_stop = 1;
  } else if (strcmp(op, "service_on") == 0) {
    shm->service_mode = 1;
    shm->emergency_mode = 0;
    shm->floor_request_pending = 0;
    shm->floor_request[0] = '\0';
  } else if (strcmp(op, "service_off") == 0) {
    shm->service_mode = 0;
  } else if (strcmp(op, "floor") == 0) {
    if (shm->service_mode != 0 || shm->emergency_mode != 0) {
      printf("Floor selection only allowed in normal operation.\n");
      return 0;
    }
    if (shm->floor_request_pending != 0) {
      printf("A floor request is already pending.\n");
      return 0;
    }
    strcpy(shm->floor_request, floor_arg);
    shm->floor_request_pending = 1;
  } else { /* up or down */
    if (shm->service_mode != 1) {
      printf("Operation only allowed in service mode.\n");
      return 0;
    }
    /* TODO: wording of the next two messages is not confirmed by the spec */
    if (strcmp(shm->status, "Between") == 0 ||
        strcmp(shm->current_floor, shm->target_floor) != 0) {
      printf("Operation not allowed while elevator is moving.\n");
      return 0;
    }
    if (strcmp(shm->status, "Closed") != 0) {
      printf("Operation not allowed while doors are open.\n");
      return 0;
    }
    if (!floor_to_int(shm->current_floor, &current) ||
        !next_floor(current, strcmp(op, "up") == 0 ? 1 : -1, &target)) {
      printf("Invalid floor specified.\n");
      return 0;
    }
    int_to_floor(target, shm->target_floor);
  }
  return 1;
}

int main(int argc, char *argv[])
{
  const char *op;
  const char *name;
  const char *floor_arg = NULL;
  int floor;
  char shm_name[64];
  int fd;
  car_shared_mem *shm;
  int changed;

  if (argc < 3) {
    printf("Usage: %s {operation} {car name} [floor]\n", argv[0]);
    return 1;
  }
  op = argv[1];
  name = argv[2];

  if (strcmp(op, "open") != 0 && strcmp(op, "close") != 0 &&
      strcmp(op, "stop") != 0 && strcmp(op, "service_on") != 0 &&
      strcmp(op, "service_off") != 0 && strcmp(op, "up") != 0 &&
      strcmp(op, "down") != 0 && strcmp(op, "floor") != 0) {
    printf("Invalid operation.\n");
    return 1;
  }
  if (strcmp(op, "floor") == 0) {
    if (argc != 4) {
      printf("Usage: %s floor {car name} {floor}\n", argv[0]);
      return 1;
    }
    floor_arg = argv[3];
    if (!floor_to_int(floor_arg, &floor)) {
      printf("Invalid floor specified.\n");
      return 1;
    }
  } else if (argc != 3) {
    printf("Usage: %s {operation} {car name}\n", argv[0]);
    return 1;
  }
  if (!valid_car_name(name)) {
    printf("Invalid car name.\n");
    return 1;
  }

  snprintf(shm_name, sizeof(shm_name), "%s%s", SHM_PREFIX, name);
  fd = shm_open(shm_name, O_RDWR, 0);
  if (fd == -1) {
    printf("Unable to access car %s.\n", name);
    return 1;
  }
  shm = mmap(NULL, sizeof(*shm), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (shm == MAP_FAILED) {
    printf("Unable to access car %s.\n", name);
    close(fd);
    return 1;
  }

  pthread_mutex_lock(&shm->mutex);
  changed = apply(shm, op, floor_arg);
  if (changed) {
    pthread_cond_broadcast(&shm->cond);
  }
  pthread_mutex_unlock(&shm->mutex);

  munmap(shm, sizeof(*shm));
  close(fd);
  return changed ? 0 : 1;
}
