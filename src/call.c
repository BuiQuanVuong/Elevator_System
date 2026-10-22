#include "common.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
  int floor;
  const char *direction;
  char msg[64];
  int fd;
  char *reply;

  if (argc != 4) {
    printf("Usage: %s {car name} {floor} {up|down}\n", argv[0]);
    return 1;
  }

  /* Validate everything before touching the network */
  if (!valid_car_name(argv[1])) {
    printf("Invalid car name.\n");
    return 1;
  }
  if (!floor_to_int(argv[2], &floor)) {
    printf("Invalid floor specified.\n");
    return 1;
  }
  if (strcmp(argv[3], "up") == 0) {
    direction = "UP";
  } else if (strcmp(argv[3], "down") == 0) {
    direction = "DOWN";
  } else {
    printf("Invalid direction specified.\n");
    return 1;
  }

  fd = connect_controller();
  if (fd == -1) {
    printf("Unable to connect to elevator system.\n");
    return 1;
  }

  snprintf(msg, sizeof(msg), "CALL %s %s %s", argv[1], argv[2], direction);
  if (send_msg(fd, msg) == -1 || (reply = recv_msg(fd)) == NULL) {
    printf("Unable to connect to elevator system.\n");
    close(fd);
    return 1;
  }
  close(fd);

  if (strcmp(reply, "ACCEPTED") == 0) {
    printf("Car %s is arriving.\n", argv[1]);
  } else if (strcmp(reply, "UNAVAILABLE") == 0) {
    printf("Sorry, that car is unavailable for this request.\n");
  } else {
    printf("Unexpected reply from elevator system.\n");
    free(reply);
    return 1;
  }
  free(reply);
  return 0;
}
