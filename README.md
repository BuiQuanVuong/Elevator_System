# Elevator Control System

A multi-process elevator simulation in C for Linux. Each elevator car runs as its own process and is managed through POSIX shared memory. A central controller schedules every car over TCP, and a separate safety process watches each car for faults.

```
call ──TCP──► controller ◄──TCP── car ◄──shm──► internal
                                   ▲
                                   └────shm────► safety
```

## Programs

| Program | What it does | Communicates via |
|---|---|---|
| `controller` | Schedules all connected cars | TCP server on `127.0.0.1:3001` |
| `car` | Runs the door and movement state machine for one car | Shared memory + TCP client |
| `call` | Presses a hall call button once, prints the result and exits | TCP client |
| `internal` | Operates the buttons and maintenance controls inside a car | Shared memory |
| `safety` | Sends a heartbeat and enforces the safety rules for one car | Shared memory |

## Building

Requires Linux (or WSL), `gcc` and `make`.

```bash
cd src
make          # builds all five programs
make clean    # removes binaries and object files
```

Compiled with `-std=c11 -Wall -Wextra -Wpedantic`, linked with `-pthread -lrt`.

## Running

Start each program in its own terminal, from `src/`:

```bash
./controller
./car A 1000 B2 10        # car "A", 1000 ms per phase, floors B2 to 10
./safety A
./call A 3 up             # someone on floor 3 wants to go up
./internal floor A 7      # a passenger in car A selects floor 7
```

Press Ctrl+C to stop `controller`, `car` or `safety`. When a car shuts down it removes its shared-memory object.

### Command reference

```
./controller
./car      {name} {delay ms} {lowest floor} {highest floor}
./safety   {car name}
./call     {car name} {floor} {up|down}
./internal {operation} {car name} [floor]
```

`internal` operations:

| Operation | Effect |
|---|---|
| `open`, `close` | Press the door open / close button |
| `stop` | Press the emergency stop button |
| `floor {floor}` | Select a destination floor (normal operation only) |
| `service_on`, `service_off` | Enter or leave service mode (this also clears emergency mode) |
| `up`, `down` | In service mode only, move one floor while the doors are closed |

### Names and floors

- **Car names** are 1–31 characters long, made of letters, digits, `_` and `-`, and must start with a letter or digit.
- **Floors** run from `1` to `999` above ground and from `B1` to `B99` below ground. There is no floor 0. `B1` is directly below `1`, and the basement numbers increase going down.

## How it works

### Shared memory (`car` ↔ `internal`, `safety`)

Each car creates a shared-memory object named `/cab403_car_{name}`, laid out as `car_shared_mem` in `src/shared.h`. The struct contains a process-shared mutex and condition variable, the car's status and floors, its button and sensor flags, its mode flags and the safety heartbeat counter. Any process that changes a field must hold the mutex and then broadcast on the condition variable.

A car's status is always one of `Opening`, `Open`, `Closing`, `Closed` or `Between` (moving between floors). Each phase lasts `delay` milliseconds.

### Network protocol (`call`, `car` ↔ `controller`)

Every message is a 32-bit big-endian length followed by that many bytes of ASCII text, with no terminator.

| Direction | Message | Meaning |
|---|---|---|
| call → controller | `CALL {car} {floor} {UP\|DOWN}` | Hall call request |
| controller → call | `ACCEPTED` / `UNAVAILABLE` | Reply; the connection then closes |
| car → controller | `CAR {name} {lowest} {highest}` | Register this car |
| car → controller | `STATUS {current} {target} {status}` | Sent whenever the state changes |
| car → controller | `SELECT {floor}` | A passenger chose a floor inside the car |
| car → controller | `SERVICE` / `EMERGENCY` | The car is leaving controller control |
| controller → car | `FLOOR {floor}` | Next stop for the car |

The controller parses messages strictly. A car that sends a malformed message is dropped. If a second car registers with a name already in use, it replaces the first.

### Scheduling

The controller uses collective control. Each car's route is a list of stops grouped into alternating up and down sweeps. A new request joins the earliest sweep it fits into without making the car turn back, and duplicate requests are merged. A call is refused (`UNAVAILABLE`) if the car isn't connected, the floor is out of its range, or the direction is impossible, such as `UP` from the top floor.

### Safety

`safety` increments the heartbeat every 100 ms. Each time the car's state changes, it applies these rules in order:

1. An obstruction while the doors are `Closing` reopens them.
2. A pressed emergency stop is reported and puts the car into emergency mode.
3. A tripped overload sensor is reported and puts the car into emergency mode.
4. Inconsistent or invalid shared-memory data is reported and puts the car into emergency mode.

The car only connects to the controller while the heartbeat is running. If the heartbeat stops for 500 ms, the car prints `Safety system disconnected! Entering emergency mode.` and enters emergency mode. In emergency mode the door buttons still work, but the car won't start any new movement.

`safety.c` is self-contained and doesn't use `shared.c`, so it can be reviewed on its own. It follows MISRA C:2012: no dynamic allocation, no stdio, and every return value is checked. The few documented deviations are listed at the top of the file.

### Concurrency

- **`car`** runs four threads that share the car's mutex and condition variable:
  - the state machine and heartbeat watchdog
  - a sender that connects to the controller and sends queued messages
  - a receiver that reads `FLOOR` commands
  - a signal thread that waits on `sigwait(SIGINT)`
- **`controller`** starts one thread per connection. A global `table_lock` protects the car table, and a per-car `send_lock` keeps each car's `FLOOR` messages in order. No mutex is ever held across a network call.
- **Timed waits** use absolute `CLOCK_MONOTONIC` deadlines, so an unrelated wake-up never stretches a phase.

## Source layout

```
src/
  Makefile
  shared.h / shared.c   shared-memory layout, floor parsing, name check, message framing
  controller.c          scheduler and TCP server
  car.c                 per-car state machine
  call.c                hall call client
  internal.c            in-car controls
  safety.c              safety monitor (self-contained)
```

## Troubleshooting

If a run is interrupted and a stray process is left holding port 3001 or a shared-memory object, clean up with:

```bash
pkill -f '\./car|\./controller|\./safety'
rm -f /dev/shm/cab403_car_*
```
