# Motor Hall capture

The ESP32-S3 firmware reads the three motor Hall signals through U5
(SN74LVC3G17). Hardware reference: dv-hardware `84d6dd0`, kart-medulla-v1.
Rubén reported U5 installed on 2026-09-19; electrical validation remains pending.

| Signal | Connector | GPIO | State bit |
|---|---|---|---|
| Hall 1 | CN7.3 | 16 | 0 |
| Hall 2 | CN2.2 | 47 | 1 |
| Hall 3 | CN2.1 | 21 | 2 |

`components/km_hall` configures inputs without internal pulls and captures both
edges with GPIO interrupts. U5 drives the inputs. The 1 kΩ resistors before U5
are series resistors, not pull-ups; open-collector motor signals require a
pull-up on that side of U5, possibly supplied by the motor controller.

Capture only observes signals. It does not command actuators, calculate speed,
determine direction or participate in safety decisions. All eight three-bit
states are accepted because the motor's Hall arrangement is not yet measured.
No changes can mean standstill, a disconnected sensor or a stuck signal.

## Telemetry

The existing `ESP_HEALTH_STATUS` frame (0x0B, 1 Hz) preserves fields 0–6 and
appends the following. Each field occupies a big-endian 32-bit integer.

| Zero-based field | Meaning |
|---|---|
| 7 | Capture initialization error; 0 means capture started, not sensor health |
| 8 | Hall state bits; -1 if capture unavailable |
| 9–11 | Observed edge counts for Hall 1/2/3, rising and falling combined |
| 12 | Milliseconds since the last observed change; -1 before any change |
| 13 | Microseconds between the last two observed changes; -1 until two changes |
| 14 | Number of observations where more than one bit changed |

Counts wrap modulo 2^32 and reset on reboot. Interpret their wire values as
unsigned. Times saturate at 2147483647. On initialization failure the counts are
zero and state/times are -1. The classic ESP32 build reports unsupported capture
without claiming its Hall/UART pins.

The snapshot is protected against concurrent interrupts. Sampling uses two GPIO
register banks, so it is not simultaneous across all three inputs. Multi-bit
changes expose some ambiguous observations, but a zero count does not prove no
pulses were missed. GPIO interrupts are not a hardware edge FIFO; maximum usable
rate must be validated against the motor. No software debounce suppresses short
pulses, so noise can also increase counts. The handler runs from instruction RAM;
if another component has already installed the GPIO interrupt service, that
service's allocation flags determine whether it remains enabled during flash work.

Appending eight fields adds 32 bytes per second (320 bit/s with serial framing).
The kart-brain receiver forwards trailing health fields. Its `dev` dashboard
implementation (`6680bce`) displays the raw Hall values and derives speed when
calibrated; on-kart validation remains pending. No new message identifier is used.
See [speed calculation, calibration and limitations](https://um-driverless.github.io/kart-docs/assembly/electronics/kart-medulla/firmware/#motor-hall-speed)
for the end-to-end explanation.

## Bench validation

1. Isolate steering and traction actuator power before flashing or opening a
   serial connection. Keep the Hall supply and ESP32 powered for the reading test.
   Port opening can pulse reset lines even though the monitor requests neither.
2. Flash the ESP32-S3 build through the usual procedure. Stop other serial-port
   users before running the monitor (the kart-brain service normally owns it).
3. Run from the firmware repository on the machine connected to the board:

   ```sh
   python3 tools/monitor_halls.py /dev/ttyACM0
   ```

   The tool requires `pyserial`, reads at 115200 baud, validates frame checksums,
   and sends no commands. Use the actual port name on another machine.
4. Check capture reports no initialization error. At rest, counts should stay
   steady. Turn the motor slowly by hand: all three states should toggle and all
   three counts should increase. The 1 Hz display is too slow to show every state;
   stop between transitions to inspect individual states.
5. Record before/after counts for one marked mechanical motor revolution in each
   direction. Check repeatability and investigate multi-bit changes or a channel
   that never changes. Verify the input/output levels at U5 if a channel is stuck.
6. Before deriving speed, measure the sequence and transitions per revolution.
   A conventional six-state cycle gives 6p combined transitions per revolution
   for p pole pairs. Vehicle speed additionally requires the drive ratio and
   wheel circumference. Validate capture at operating speed separately.

This monitor is the Hall diagnostic tool; the older root `monitor_serial.py`
expects obsolete text output and cannot decode these binary frames.
