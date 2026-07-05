# AirIron Controller / Panasonic PACi UART Protocol Notes

> **Scope: Panasonic PACi only.** These notes and the `panasonic_paci` component target
> Panasonic **PACi** commercial/semi-commercial systems (CZ-RTC controller bus). They are
> **not** for Panasonic Etherea/residential split systems, Aquarea, VRF/ME, or any non-PACi
> product. Other Panasonic lines use different protocols and are out of scope.

Status: reverse-engineering notes, not an official Panasonic specification.  
Scope: Panasonic **PACi** indoor unit **S-100PF1E5A**/**S-100PE1R5A** with outdoor unit **U-100PEY1E5**/**U-100PE1R5A**, DNSK-P11/CN-WLAN/CN-CNT style UART traffic, ESPHome component `panasonic_paci`.  
Last updated: 2026-07-05.

## 1. Hardware / UART

Observed / confirmed PACi setups:

- Indoor unit: `S-100PF1E5A`, outdoor unit: `U-100PEY1E5`
- Indoor unit: `S-100PE1R5A`, outdoor unit: `U-100PE1R5A`
- ESP: ESP32-C3
- Hardware reference: Ingenious Makers P11 (`https://www.ingeniousmakers.com/p11`)
- UART:
  - baud: `2400`
  - data bits: `8`
  - parity: `EVEN`
  - stop bits: `1`
  - ESP TX: GPIO21
  - ESP RX: GPIO20

Hard rule:

- ESP must transmit only as source `E0`.
- ESP must never transmit frames whose first byte/source is `40`.
- `40` is the wired remote/controller source and must be treated as RX/passive traffic only.

## 2. Frame shape

Most observed frames follow this broad layout:

```text
[SRC] [DST] [OP] [LEN] [PAYLOAD ...] [XOR]
```

Examples:

```text
E0 00 15 04 08 07 00 31 CF
00 E0 18 03 80 07 00 7C
40 00 15 02 08 0F 50
00 40 18 09 80 0F 74 76 73 76 32 25 05 CB
```

Known/observed sources:

```text
E0  ESP/controller adapter source, used by this component
40  wired wall controller source, RX/passive only for ESP
00  response origin/destination field in many response frames
FE  broadcast/status-ish source/target used in main status frames
C0  extended identity response source, outdoor identity
52  compact event-ish source
F0  appears in cold-init/handshake-related traffic
```

Checksum:

```text
checksum = XOR of all previous frame bytes
```

Example:

```text
E0 00 15 04 08 07 00 31 CF
```

`CF` is XOR of:

```text
E0 ^ 00 ^ 15 ^ 04 ^ 08 ^ 07 ^ 00 ^ 31
```

Important exception / parser note:

- Some extended identity frames from `C0` were observed as checksum-less or not matching the normal rule.
- Current parser has special handling for extended identity frames, for example outdoor model/serial.

## 3. Common operation bytes

Observed operation bytes:

```text
0x11  write/control/admin write
0x15  read/request
0x17  extended read/request
0x18  response
0x1A  extended response
0x55  status/control request-ish
0x58  main status response
```

These names are inferred from observed behavior, not official Panasonic names.

## 4. Cold init / handshake notes

Cold init is partly understood and works well enough.

Observed cold init/request examples:

```text
E0 F0 15 02 00 0D 0A
E0 00 17 06 08 80 EF 00 38 00 AE
E0 00 15 02 08 0A F5
E0 00 55 04 08 81 00 46 7E
```

Observed responses:

```text
00 E0 18 12 80 0D 08 00 FE FE FE FE FE FE FE FE FE FE FE FE FE FE 6F
00 E0 1A 05 80 EF 80 00 A2 B2
00 E0 18 0D 80 0A 00 2F 0F 82 6A 82 66 82 6A 7C 68 AF
00 FE 58 0B 80 81 4D 4C 00 00 76 76 E9 00 01 45
```

Do not treat residential Panasonic/DomiStyle/`5A` protocol logic as authoritative. This component is based on real PACi DNSK-P11/CN-WLAN/CN-CNT UART captures.

## 5. Main climate status

Main status can be observed in at least two lengths:

- `LEN=0x0B`: base climate status payload.
- `LEN=0x10`: base climate status payload plus an extra tail. The extra tail is currently ignored.

The decoder reads the known offsets after `80 81` and ignores unknown trailing bytes.

Main status frame examples:

```text
00 FE 58 0B 80 81 4D 4C 00 00 76 75 E9 00 01 46
00 FE 58 0B 80 81 4D 4C 00 00 76 76 E9 00 01 45
00 40 18 0B 80 81 4D 4C 00 00 76 77 E9 00 01 BA
```

The parser currently extracts at least:

- power
- mode
- fan
- target temperature
- current/room temperature
- action
- preset/Eco state

Observed decoded examples:

```text
Status: power=on mode=2 fan=2 target=24.0 current=23.5
Status: power=on mode=2 fan=2 target=24.0 current=24.0
Status: power=on mode=2 fan=2 target=24.0 current=24.5
```

Mode values confirmed working on the target unit:

```text
mode=1  HEAT
mode=2  COOL
mode=3  FAN_ONLY
mode=4  DRY
mode=5  AUTO / heat-cool auto
```

Some captures may report a neighbouring auto-related value. Treat that as PACi-specific
status behaviour until more units are tested.

Fan writes are confirmed on the target unit for auto/high/medium/low. The write payload
shape differs by mode, so the implementation keeps mode-specific fan slots.

## 6. Temperature fields

### 6.1 Target/current temperature in main status

Observed status payload section:

```text
... 76 75 ...
... 76 76 ...
... 76 77 ...
```

Decoded examples:

```text
target=24.0 current=23.5
target=24.0 current=24.0
target=24.0 current=24.5
```

On the target PACi unit, these status bytes encode target/current temperature with 0.5°C granularity.

### 6.2 Outdoor temperature: final working source is EF/21, not 0F

Important correction:

- `00 40 18 09 80 0F ...` was previously misread as outdoor/controller temperature.
- This frame must **not** be used as authoritative outdoor temperature.
- It is a status-like block and contains multiple temperature-looking bytes, but it does not track the app’s outdoor value reliably.

Observed `0F` request/response:

```text
40 00 15 02 08 0F 50
00 40 18 09 80 0F 74 76 73 76 32 25 05 CB
```

Old wrong candidates from that frame:

```text
b0=0x74 -> 29.5
b1=0x76 -> 30.5
b2=0x73 -> 29.0
b3=0x76 -> 30.5
```

This caused false matches and offset guessing. Do not use this as the outdoor temperature source.

Correct outdoor-unit reported temperature source:

Request, as sent by native controller:

```text
40 00 17 07 08 80 EF 00 21 00 20 36
```

Equivalent ESP request must use source `E0`:

```text
E0 00 17 07 08 80 EF 00 21 00 20 96
```

Response examples:

```text
00 40 1A 07 80 EF 80 00 21 01 35 A7
00 E0 1A 07 80 EF 80 00 21 01 35 07
00 40 1A 07 80 EF 80 00 21 01 22 B0
00 40 1A 07 80 EF 80 00 21 01 20 B2
00 40 1A 07 80 EF 80 00 21 01 1E 8C
```

Decoded as big-endian tenths of °C:

```text
0x0135 = 309 -> 30.9°C
0x0122 = 290 -> 29.0°C
0x0120 = 288 -> 28.8°C
0x011E = 286 -> 28.6°C
```

Current parser rule:

```text
Match:
00 E0/40 1A 07 80 EF 80 00 21 HH LL CS

raw_tenths = (HH << 8) | LL
outdoor_temperature = raw_tenths / 10.0
```

Passive wired-controller/native updates:

- When the wired controller/native adapter asks `40 00 17 07 08 80 EF 00 21 00 20 36`, the response arrives shortly after.
- The ESP passively parses `00 40 1A 07 80 EF 80 00 21 HH LL CS` and updates Home Assistant immediately.
- The ESP's own `E0...EF/21` read is used as fallback polling.

Recommended polling:

- First read after boot: a few seconds after init/identity reads.
- Periodic fallback: every few minutes, not aggressive.


## 6.3 Fan write slots

Fan commands use explicit set writes, not toggles. The current target temperature raw
value is included in the fan payload.

Confirmed options on the target PACi unit:

```text
AUTO
HIGH
MEDIUM
LOW
```

The implementation keeps mode-specific write slots because PACi captures showed that
fan payloads are not identical across all modes.


## 7. Identity reads

### 7.1 Indoor model

Request:

```text
E0 00 15 02 08 08 F7
```

Response example:

```text
00 E0 18 14 80 08 53 2D 31 30 30 50 46 31 45 35 41 20 20 20 20 20 00 D2 CF
```

ASCII:

```text
S-100PF1E5A
```

### 7.2 Indoor serial

Request:

```text
E0 00 15 02 08 0B F4
```

Response example:

```text
00 E0 18 14 80 0B 31 36 30 31 31 35 47 30 33 30 30 31 32 35 20 20 00 D2 C5
```

ASCII:

```text
160115G0300125
```

### 7.3 Outdoor model

Request:

```text
E0 00 17 05 08 80 EF 00 08 9D
```

Response example:

```text
00 C0 1A 17 80 EF 80 00 08 55 2D 31 30 30 50 45 59 31 45 35 20 20 20 20 20 00 11
```

ASCII:

```text
U-100PEY1E5
```

Parser note:

- Outdoor identity responses may arrive as normal `00 E0 1A ... CS` frames or as
  checksum-less `00 C0 1A ...` frames.
- The parser accepts checksum-less `C0` only for known PACi identity response shapes
  (`EF/08` model and `EF/0B` serial) with printable ASCII payload.

### 7.4 Outdoor serial

Request:

```text
E0 00 17 05 08 80 EF 00 0B 9E
```

Response examples:

```text
00 C0 1A 15 80 EF 80 00 0B 31 35 31 31 31 37 47 36 38 37 30 30 31 39 20 20
00 E0 1A 15 80 EF 80 00 0B 31 35 31 31 31 37 47 36 38 37 30 30 31 39 20 20 7F
```

ASCII:

```text
151117G6870019
```

## 8. Admin settings

Admin settings are not boolean switches. They are service/admin enum-like settings and should be represented as `select` entities in HA.

Current admin codes:

```text
0x31  ventilation output
0x32  room/remote temperature sensor source
0x33  temperature unit Celsius/Fahrenheit
```

### 8.1 Admin read

Important correction:

Old wrong format:

```text
E0 00 15 03 08 07 KK CS
```

Correct format:

```text
E0 00 15 04 08 07 00 KK CS
```

Known read requests:

```text
E0 00 15 04 08 07 00 31 CF
E0 00 15 04 08 07 00 32 CC
E0 00 15 04 08 07 00 33 CD
```

Response is value-only:

```text
00 E0 18 03 80 07 VV CS
```

or passive wired-remote form:

```text
00 40 18 03 80 07 VV CS
```

Because the response contains only `VV`, the parser maps it through the current
in-flight admin read transaction:

```text
in-flight code=0x31 + VV -> ventilation output
in-flight code=0x32 + VV -> room temp sensor source
in-flight code=0x33 + VV -> temperature unit
```

Observed examples:

```text
E0 00 15 04 08 07 00 31 CF
00 E0 18 03 80 07 00 7C

E0 00 15 04 08 07 00 32 CC
00 E0 18 03 80 07 01 7D

E0 00 15 04 08 07 00 33 CD
00 E0 18 03 80 07 00 7C
```

### 8.2 Admin write

Write request shape:

```text
E0 00 11 04 08 07 KK VV CS
```

Observed passive/native writes:

```text
40 00 11 04 08 07 32 01 69
40 00 11 04 08 07 33 00 69
40 00 11 04 08 07 31 00 6B
```

ACK observed:

```text
00 40 18 02 80 A1 7B
```

ESP must send write with source `E0`, never `40`.

### 8.3 Passive admin update handling

When the app/native controller changes an admin setting, the bus can contain full write/update form:

```text
40 00 11 04 08 07 KK VV CS
```

Parser should immediately apply:

```text
if KK in {31,32,33}:
  update admin setting KK to VV
```

This is already working in current context. Logs showed:

```text
Passive admin setting update: code=0x32 value=0x01
Passive admin setting update: code=0x33 value=0x00
Passive admin setting update: code=0x31 value=0x00
```

### 8.4 Admin select mappings

Current intended HA entities:

```yaml
ventilation_output_select:
  name: Panasonic AC Ventilation Output

remote_temperature_sensor_select:
  name: Panasonic AC Room Temperature Sensor

temperature_unit_select:
  name: Panasonic AC Temperature Unit
```


Options:

```text
Ventilation Output:
  0x00 -> Not Connected
  0x01 -> Connected

Room Temperature Sensor:
  0x00 -> Main Unit
  0x01 -> Remote Controller

Temperature Unit:
  0x00 -> Celsius
  0x01 -> Fahrenheit
```

### 8.5 Admin read reliability

Observed problem:

- Admin read responses sometimes do not arrive.
- If a read is missed, setting can remain `unknown`.

Current desired retry behavior:

```text
For each code 31/32/33:
  send read
  wait response timeout
  if no response:
    retry same code
  max retries: 3
  only after success or final failure move to next code
```

Important state-machine rule:

- The admin code is stored on the in-flight transaction, not in a global pending variable.
- The transaction is considered in-flight only after the read frame is actually transmitted.
- Value-only responses are ignored unless a matching `WaitAdminValue` transaction is active.

Current intended constants:

```text
ADMIN_READ_RESPONSE_TIMEOUT ~= 1200 ms
ADMIN_READ_RETRY_DELAY      ~= 1200 ms
ADMIN_READ_MAX_RETRIES      = 3
```

After admin write:

- Schedule forced read-back.
- Do not rely only on ACK.
- ACK means the unit accepted/acknowledged traffic, not necessarily that HA state is now verified.

## 9. Command writes and ACK retry

Command writes sometimes appear to be ignored or the ACK is missed.

ACK retry model:

```text
If a frame is sent as a WaitAck transaction:
  store it as current_transaction_ (with its attempt count)
  wait for the ACK
  if timeout:
    retry the same frame
  max retries: 3
  retry gap: ~350 ms
```

Observed ACK:

```text
00 40 18 02 80 A1 7B
```

ACK matching:

- The ACK matcher requires the shape `00 .. 18 02 80 A1` and an active `WaitAck`
  transaction.
- A passive `80 A1` seen outside a `WaitAck` transaction is not accepted as confirmation.
- This is intentionally conservative, but it is still based on observed PACi behaviour,
  not an official ACK correlation field.

Retry safety:

- All writes are explicit set operations (set power / mode / target temperature /
  fan / admin value), never toggles, so retrying cannot flip state back and forth.

Command coalescing (per command class):

- Coalescing is done by command class, not as one broad "control write" bucket.
- A new **mode** command purges queued POWER / MODE / TEMPERATURE / FAN writes
  (the fresh mode call re-enqueues the sequence it needs).
- A new **target temperature** purges only queued TEMPERATURE writes.
- A new **fan** command purges only queued FAN writes.
- Eco and admin writes are never coalesced (independent settings).
- A frame already in flight (`current_transaction_`) is never interrupted; it
  completes or exhausts its retries first. Reads (identity/admin/outdoor) are preserved.

This avoids the trap where a fast temperature/fan update would delete a still-needed
POWER_ON/MODE sequence from the previous command.

## 10. Admin select state / unknown handling

Background:

- Originally the goal was to show admin selects as "unavailable" in HA until the real
  value was read. ESPHome `select` has no working public API for a runtime per-entity
  unavailable state (`set_has_state(false)` + `status_set_error()` still showed an
  active `unknown` dropdown, and `state_callback_` expects an option index, not a
  string), so that UI behavior was abandoned.

Current behavior:

- The selects are plain `select` entities. There is no fake "unavailable" placeholder.
- Per-setting "known" flags (`ventilation_output_known_`, `remote_temperature_sensor_known_`,
  `fahrenheit_unit_known_`) are set the first time a real value is read from the bus
  (or echoed by a passive write).
- The value is published to the select as soon as it is known, so HA reflects the real
  state shortly after boot.
- Write handling uses the known flag to stay safe:
  - If the current value is **not yet known**, a user write is **ignored** and an admin
    read is scheduled instead (do not send a blind toggle when the current state is
    unknown). The user can set it again once the value has loaded.
  - If the value **is known** and the request matches it, the select is just re-published
    (no redundant bus write).
  - If the value is known and differs, the write is sent and then verified by read-back
    (Section 8).

Rationale: a correct value arriving shortly after boot, plus read-back verification and
a guard against blind toggles, is more useful than a non-functional "unavailable" placeholder.

## 11. Outdoor temperature update behavior

Passive update path works:

When native app/controller asks:

```text
40 00 17 07 08 80 EF 00 21 00 20 36
```

Response arrives quickly:

```text
00 40 1A 07 80 EF 80 00 21 HH LL CS
```

The ESP parser updates Home Assistant immediately on passive `00 40 ... EF/21` responses.

ESP fallback polling:

```text
E0 00 17 07 08 80 EF 00 21 00 20 96
```

Useful after boot and when app is not active.

## 12. Misc observed frames

### 12.1 Compact event

Observed:

```text
00 52 11 04 80 86 44 01 04
```

Current parser logs:

```text
Compact event key=0x44 value=0x01
```

Meaning unknown.

### 12.2 Keepalive/status-ish frame

Observed repeatedly:

```text
00 FE 10 02 80 8A E6
```

Meaning unknown. Likely keepalive/status heartbeat.

### 12.3 Unknown read/response 0x58

Observed:

```text
40 00 15 04 08 58 00 11 10
00 40 18 03 80 58 00 83
```

Meaning unknown.

### 12.4 Unknown read/response 0x0C

Observed:

```text
40 00 15 06 08 0C 80 00 00 48 9E
00 40 18 08 80 0C 80 03 00 00 48 00 17

40 00 15 06 08 0C 80 00 00 4E 99
00 40 18 07 80 0C 00 00 00 4E 00 9D
```

Meaning unknown.

### 12.5 Unknown read/response 0x10

Observed after `0F` in native query sequence:

```text
40 00 15 02 08 10 4F
00 40 18 05 80 10 00 35 33 CB
```

Meaning unknown.

### 12.6 Extended EF/21 outdoor temperature response

Observed:

```text
40 00 17 07 08 80 EF 00 21 00 20 36
00 40 1A 07 80 EF 80 00 21 01 35 A7
```

This is the current outdoor-unit reported temperature source.

## 13. Parser robustness notes

There are frequent unsynchronized bytes in logs:

```text
Dropping unsynchronized byte: ...
Dropping N byte(s) before incomplete frame
```

Likely causes:

- echo of our own TX
- concurrent wired-controller/native-adapter traffic
- checksum-less/short PACi frames around the normal framed traffic
- still-unknown frame families

Design implications:

1. Do not spam the bus.
2. Keep admin reads rare and retry controlled.
3. Avoid overlapping identity/outdoor/admin reads too tightly.
4. Do not assign value-only responses unless in-flight transaction state is reliable.
5. Passive parsing should accept both `00 E0 ...` and `00 40 ...` response forms where appropriate.
6. Never transmit as `40`.
7. For checksum-less `C0` outdoor identity frames, wait for the full candidate frame instead of skipping ahead to a later complete frame.

## 14. Current known-good entities

Climate:

```text
AirIron Controller
```

Sensors:

```text
Panasonic AC Target Temperature
Panasonic AC Current Temperature
Panasonic AC Outdoor Temperature
```

Text sensors:

```text
Panasonic AC Indoor Model
Panasonic AC Indoor Serial
Panasonic AC Outdoor Model
Panasonic AC Outdoor Serial
```

Admin selects:

```text
Panasonic AC Ventilation Output
Panasonic AC Room Temperature Sensor
Panasonic AC Temperature Unit
```

Diagnostics:

```text
AirIron Controller WiFi Signal
AirIron Controller IP Address
AirIron Controller WiFi SSID
AirIron Controller WiFi BSSID
```

## 15. Status of mappings and open questions

Confirmed working on the target unit (S-100PF1E5A / U-100PEY1E5):

- Mode mapping for all modes: heat, cool, fan only, dry, auto.
- Fan mode writes across all modes (auto/high/medium/low).
- Power, target temperature, and eco/powersave control.
- Admin read/write for codes `31`/`32`/`33` with read-back.
- Outdoor temperature via EF/21.

Settled / implemented (not open):

- ACK matching is conservative: the matcher requires the `00 .. 18 02 80 A1` frame
  shape and an in-flight `WaitAck` transaction. A passive ACK outside `WaitAck` is
  ignored. A same-shape ACK inside the `WaitAck` window can still be accepted because
  no confirmed correlation id is known.
- All outgoing frames are serialized through the TX queue. Admin reads additionally
  use a `WaitAdminValue` transaction because their responses are value-only. Identity
  and outdoor reads are fire-and-forget and are matched by response frame shape. A
  value-only admin response is attributed via the code carried on the in-flight
  transaction, not a global pending variable.
- Custom `select` "unavailable while unknown" UI was dropped (no working ESPHome
  API). Selects publish on read; writes are guarded by per-setting "known" flags and
  verified by read-back.

Still open / not byte-mapped:

1. Meaning of the `0F` status block (multiple temperature-looking bytes; not the
   authoritative outdoor temperature source).
2. Meaning of `0C`, `10`, `58`, and the `44` compact event frames.
3. Whether outdoor EF/21 is outdoor air temperature, outdoor coil/sensor, or another
   outdoor-unit-reported sensor. Current working name remains `Outdoor Temperature`.

## 16. Minimal implementation rules

Do:

```text
TX source: E0 only
Parse passive 40 frames
Checksum normal frames by XOR
Use EF/21 for outdoor temperature
Use 08 07 00 KK for admin reads
Attribute value-only admin responses via the in-flight transaction's code
Retry admin reads on missing response
Read back admin writes
Retry ACK-backed commands carefully
```

Do not:

```text
Do not transmit 40
Do not use old 5A/DomiStyle handshake as authoritative
Do not parse 0F as outdoor temperature
Do not treat admin settings as boolean switches
Do not optimistically trust admin write without read-back
Do not attach a value-only admin response unless a matching `WaitAdminValue` transaction is active
```


## Current multi-device parser notes

The current parser intentionally handles only stable state sources and known transactions.

Confirmed variants from PACi logs:

- Main climate status appears as both `00 FE 58 0B ... 80 81 ...` and `00 FE 58 10 ... 80 81 ...`. Known offsets are decoded; extra tail bytes are ignored.
- Admin read responses appear as both `80 07 VV` and `80 07 VV KK`. If the code `KK` is present it is used directly; otherwise the pending admin transaction supplies the code.
- Outdoor-unit temperature is confirmed via extended `EF/21`. Both ESP-requested `00 E0 1A ... EF ... 21 HH LL` and wired-controller-passive `00 40 1A ... EF ... 21 HH LL` responses are accepted.
- Wired-controller `40 00 55 ...` status polls, auxiliary block `0x0C`, and auxiliary block `0x21` are recognized as known-unused traffic and are not used as HA state.
- ESP TX echo frames are ignored.

Known-unused frames are valid bus traffic. They are logged at very-verbose level rather than reported as protocol errors.
