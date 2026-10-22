#include "common.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/* ---------- Floors ---------- */

int floor_to_int(const char *s, int *out)
{
  int sign = 1;
  int value = 0;
  int i = 0;

  if (s[0] == 'B') {
    sign = -1;
    i = 1;
  }
  /* First digit must be 1-9: rejects "", "0", leading zeroes */
  if (s[i] < '1' || s[i] > '9') {
    return 0;
  }
  for (; s[i] != '\0'; i++) {
    if (s[i] < '0' || s[i] > '9') {
      return 0;
    }
    value = value * 10 + (s[i] - '0');
    if (value > 999) {
      return 0;
    }
  }
  if (sign == -1 && value > 99) {
    return 0;
  }
  *out = sign * value;
  return 1;
}

void int_to_floor(int floor, char buf[4])
{
  if (floor < -99 || floor > 999 || floor == 0) {
    buf[0] = '\0';
  } else if (floor < 0) {
    snprintf(buf, 4, "B%d", -floor);
  } else {
    snprintf(buf, 4, "%d", floor);
  }
}

int next_floor(int floor, int dir, int *out)
{
  int next = floor + dir;

  if (next == 0) {
    next += dir; /* there is no floor zero: 1 <-> B1 */
  }
  if (next < -99 || next > 999) {
    return 0;
  }
  *out = next;
  return 1;
}

/* ---------- Car names ---------- */

int valid_car_name(const char *s)
{
  size_t len = strlen(s);

  if (len < 1 || len > 31) {
    return 0;
  }
  for (size_t i = 0; i < len; i++) {
    char c = s[i];
    int alnum = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    if (i == 0 && !alnum) {
      return 0;
    }
    if (!alnum && c != '_' && c != '-') {
      return 0;
    }
  }
  return 1;
}

/* ---------- Network messages: 32-bit length prefix + ASCII ---------- */

static int send_all(int fd, const void *buf, size_t len)
{
  const char *p = buf;

  while (len > 0) {
    ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
    if (n == -1) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
    p += n;
    len -= (size_t)n;
  }
  return 0;
}

static int recv_all(int fd, void *buf, size_t len)
{
  char *p = buf;

  while (len > 0) {
    ssize_t n = recv(fd, p, len, 0);
    if (n == 0) {
      return -1; /* disconnected */
    }
    if (n == -1) {
      if (errno == EINTR) {
        continue;
      }
      return -1;
    }
    p += n;
    len -= (size_t)n;
  }
  return 0;
}

int send_msg(int fd, const char *msg)
{
  size_t len = strlen(msg);
  uint32_t prefix = htonl((uint32_t)len);

  if (send_all(fd, &prefix, sizeof(prefix)) == -1) {
    return -1;
  }
  return send_all(fd, msg, len);
}

char *recv_msg(int fd)
{
  uint32_t prefix;
  uint32_t len;
  char *buf;

  if (recv_all(fd, &prefix, sizeof(prefix)) == -1) {
    return NULL;
  }
  len = ntohl(prefix);
  if (len > MAX_MSG_LEN) {
    return NULL;
  }
  buf = malloc(len + 1);
  if (buf == NULL) {
    return NULL;
  }
  if (recv_all(fd, buf, len) == -1) {
    free(buf);
    return NULL;
  }
  buf[len] = '\0';
  return buf;
}

int connect_controller(void)
{
  struct sockaddr_in addr;
  int fd = socket(AF_INET, SOCK_STREAM, 0);

  if (fd == -1) {
    return -1;
  }
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(CONTROLLER_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
    close(fd);
    return -1;
  }
  return fd;
}
