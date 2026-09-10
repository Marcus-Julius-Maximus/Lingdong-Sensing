# A Low-Power Pet Sensing & Positioning System on OpenHarmony

> An integrated software-hardware build: high-precision filtered tracking, long endurance in extreme scenarios, intelligent all-environment caretaking.

`OpenHarmony` · `Lightweight MQTT` · `Bidirectional Correction`

## Project Background & Stages

Built on the OpenHarmony IoT chip platform Hi3863 as the core control unit, this project designs a low-power wearable for outdoor pet tracking and health monitoring. The hardware integrates an independent GNSS positioning module, a six-axis inertial measurement unit (IMU), a contact high-precision NTC body-temperature sensor, and a 4G cellular communication module. The system covers the full chain end to end — from the sensor side to firmware, from the communication link to cloud parsing, from data aggregation to App display.


## Table of Contents

- [1. Hardware Platform & Sensor Selection](#1-hardware-platform--sensor-selection)
- [2. Device Communication Protocol Design](#2-device-communication-protocol-design)
- [3. IMU Data Acquisition Strategy: Two Modes in Parallel](#3-imu-data-acquisition-strategy-two-modes-in-parallel)
- [4. The Coordinate-Processing Chain: WGS-84 to GCJ-02](#4-the-coordinate-processing-chain-wgs-84-to-gcj-02)
- [5. Full-Chain Low-Power Design](#5-full-chain-low-power-design)
- [6. Device Full-Lifecycle Management](#6-device-full-lifecycle-management)
- [7. User System & Device-Binding Authentication](#7-user-system--device-binding-authentication)
- [8. Smart-Home Linkage](#8-smart-home-linkage)
- [9. AI Assistant Integration](#9-ai-assistant-integration)

## 1. Hardware Platform & Sensor Selection

### 1.1 Main Controller: Hi3863

The HiSilicon Hi3863 is a RISC-V wireless SoC for mid-to-low-end IoT applications, integrating 2.4 GHz Wi-Fi (802.11 b/g/n) and BLE 5.2 with on-chip PA and RF matching network. The Hi3863's RAM ceiling is on the order of 200 KB — not enough to run the full runtime stack of libmosquitto or paho-embedded-c. This directly dictates the direction of the communication solution below.

### 1.2 Sensor Combination

For positioning we chose an independent GNSS module over cellular base-station positioning or Wi-Fi fingerprinting. Cellular base-station positioning lands within roughly 50–300 m accuracy among city blocks; Wi-Fi fingerprinting works indoors but fails completely in outdoor areas without Wi-Fi coverage — yet the core scenarios of this project include track accuracy while the cat moves in open outdoor spaces. Cold-start first-fix time for the standalone GNSS is about 30–45 s, hot start about 2–5 s; since the device does not position continuously in real time (each report is intermittent), the cold-start latency is acceptable.

Motion sensing uses a six-axis IMU (tri-axis accelerometer + tri-axis gyroscope). The IMU serves two independent purposes: primary data source for step counting and motion-state recognition, and the trigger source for low-power wakeup — in deep sleep the CPU sleeps and 4G is powered off, while the IMU keeps drawing microamp-level current to continuously detect acceleration changes; when `delta_g` exceeds the threshold (about 1.2 m/s²) it wakes the main controller via a hardware interrupt. Body temperature uses the contact NTC scheme, computing temperature from the thermistor's voltage-divider value. NTC accuracy is about ±0.3°C, and response speed depends on contact tightness with the measured surface. As a reference, the ambient temperature sensor is independent of the NTC, used to compute temperature/humidity and to help correct the NTC's contact error.

Communication uses a 4G Cat.1 module rather than NB-IoT or Cat.M — because the device must support real-time bidirectional communication (device reporting + cloud configuration push). NB-IoT downlink latency typically runs 5–15 s, failing the low-latency requirement of configuration changes. Cat.1 keeps downlink latency within 1–2 s, but peak power draw must be constrained by an overall power-management strategy.

### 1.3 The `pet_status` Table: A Direct Mirror of Hardware Data

The `pet_status` table is the direct database mirror of the hardware sensor data. The latitude field uses `decimal(10,7)` precision — 7 decimal places correspond to roughly 0.01 m of planar positioning accuracy, enough to cover the civilian precision ceiling of the GPS C/A code. The `motion_status` field uses a `tinyint` encoding:

| Encoding | Meaning |
| ---- | ---- |
| 2 | Stationary / sleeping |
| 1 | Walking / light motion |
| 3 | Jumping or vigorous motion |

The encoding is decided jointly by the IMU's `delta_g` and `gyro_energy`. NTC temperature and the related fields carry independent `decimal(4,1)` precision, one decimal place corresponding to 0.1°C display resolution. The device's unique identifier `device_id` serves as the primary key, burned into the module's NVRAM at the factory, and auto-provisioned by the Omega side on first report (`_init_device_if_not_exists`).

## 2. Device Communication Protocol Design

### 2.1 A Custom Lightweight MQTT

The Hi3863's RAM cannot run a full MQTT client library — paho-embedded-c's MQTT protocol encoding tables alone take over 15 KB of firmware, and with the TCP/IP stack and Wi-Fi driver, heap space left for the application layer is extremely tight. So the communication layer constructs a minimal viable MQTT 3.1.1 implementation directly on the chip's TCP Socket interface. The device implements only three packet types:

| Packet type | Fixed header | Purpose |
| ---- | ---- | ---- |
| CONNECT | `0x10` | Declaring the Client ID and Clean Session flag on first connect |
| PUBLISH | `0x30` | Uploading sensor data |
| PINGREQ | `0xC0 00` | Keeping the heartbeat alive |

No SUBSCRIBE — the device subscribes to no downlink topic; all configuration commands are triggered by Alpha's 3-second poll, and device-side configuration updates are reported after a physical dial-wheel switch on the device, confirmed by the cloud.

The heartbeat is a PINGREQ every 30 seconds — the value lives in the `keepAlive` field of `GlobalDataManager.ts`. If the cloud receives no heartbeat or data packet within 30 seconds, it marks the device offline. The reconnect logic lives in the device firmware: on detecting a TCP drop it resets the socket handle, re-runs DNS resolution and the CONNECT handshake sequence, retrying every 3 seconds.

### 2.2 Data Report Packet Structure

Each report is a JSON object with up to three substructures:

- **`base`**: the basic sensor data — battery percentage (0–100), ambient temperature (°C), NTC body temperature (°C), and ambient humidity (%).
- **`imu`**: different motion data depending on the current upload mode — instantaneous tri-axis acceleration and angular velocity (one float each) in Mode 0, or the mean motion features of the previous communication cycle in Mode 1.
- **`gps`**: exists only when a valid fix is present, holding longitude and latitude floats.

The `device_id` field must exist in every packet, serving as the basis for Omega-side routing and `pet_status` primary-key writes. After Omega validates and writes the packet and updates the `beta_status` fields, the full sync payload is pushed to the App over `testtopic/1`.

> [!NOTE]
> The device's JSON reports contain no nesting — Omega's parsing is a single flat level, never recursive. This isn't a performance decision; it exists to simplify assembly in the device code: on the Hi3863 the JSON string is built line-by-line with printf, and nesting means inserting comma and newline control inside loops — a higher error rate than a flat layer.

### 2.3 Device Configuration Sync

The device has only two configuration items: `upload_interval` (default 30000 ms) and `upload_mode` (default 0). Three paths modify configuration:

1. **The hardware dial wheel** — the user flips a physical switch on the device, which resets local parameters and reports the new interval and mode over MQTT; Alpha writes them into the devices table and flags the shadow cache.
2. **The App settings** — the user edits parameters in the App's config page, the PHP interface writes the devices table, and Alpha's 3-second poll detects the change and pushes it.
3. **Automatic sync on device online** — when the device first connects or reconnects, Alpha's `/online` handler detects the online event, pulls the current config straight from the devices table, and immediately pushes one frame.

All three paths converge on the same devices table, with write priority determined by the database's last-write time.

## 3. IMU Data Acquisition Strategy: Two Modes in Parallel

### 3.1 Mode 0: Instantaneous Value Upload

In Mode 0, the IMU samples tri-axis acceleration (ax, ay, az) and tri-axis angular velocity once at the end of each 3-second cycle, encoding and uploading the raw values. The Omega side runs the full `PetAlgorithms.analyze_motion_latest()` on receipt. The function first computes `a = sqrt(ax²+ay²+az²)`, the acceleration vector magnitude, then `delta_g = |a − 9.80665|`, the deviation from gravitational acceleration:

| Classification condition | Motion state |
| ---- | ---- |
| `delta_g` above 4.5 m/s² or total gyro energy above 150 | Jumping or vigorous motion (status=3) |
| `delta_g` in 1.2–4.5 or gyro energy in 25–150 | Walking (status=1) |
| `delta_g` below 1.2 with gyro energy below 25 | Stationary (status=2) |

From parsing the `data['imu']` field to assigning status, all processing completes inside Omega's `on_message` callback, bypassing any message queue.

### 3.2 Mode 1: Feature-Mean Upload

Mode 1's data flow is completely different from Mode 0's. The device continuously samples IMU data within one upload cycle (at a frequency higher than the upload interval) and computes five features at cycle end: mean `delta_g` (carried in the ax field), peak `delta_g` (ay field), active ratio (active sample count / total sample count, az field), mean gyro energy (gx field), and peak gyro energy (gy field). On receipt, Omega completes three computations in sequence: threshold classification of motion state using `max_delta_g` and `max_gyro`; derivation of the current cycle's active and rest durations from the active ratio and motion state; and dynamic cadence from a linear combination of undiluted gyro energy and `delta_g`, multiplied by active duration to yield the cycle step count.

> [!NOTE]
> Mode 1 joined a full iteration later than Mode 0 — the project first ran the whole chain on Mode 0, and only after upload bandwidth pressure exceeded expectations was the on-device pre-computation logic added. The real reason both modes coexist is not a technical trade-off but different user scenarios: some would rather trade battery for more precise motion recognition (Mode 0); others want one charge to last longer and don't care about the accuracy of motion details (Mode 1).

### 3.3 Step Counting: IMU Estimation vs. GPS Distance Cross-Validation

After each data report is processed, Omega computes two step counts — the IMU-derived count and the GPS-distance count — and takes the larger.

- **IMU side**: dynamic cadence = (gyro energy × 0.5) + (`delta_g` × 10.0), clamped to the 15–180 steps/min range. The coefficients 0.5 and 10.0 come from cat motion data collection and parameter search.
- **GPS side**: Haversine(distance between two fixes) ÷ `CAT_STRIDE` (0.25 m/step).

Two drift filters — distances under 2 m discarded as noise, over 500 m discarded as positioning jumps — run before the Haversine computation. The implicit assumption of this dual-channel design is that IMU and GPS error sources are uncorrelated: IMU error comes from sensor noise and body-pose changes, GPS error from satellite signal reflection and atmospheric delay; the two never deviate in the same direction at the same time, so taking the larger value reflects true steps better than averaging.

## 4. The Coordinate-Processing Chain: WGS-84 to GCJ-02

The GPS coordinates reported by the hardware are raw WGS-84 values — the internationally standard coordinate system output directly by the GNSS module. On receipt, Omega writes them verbatim into the `pet_status` table's `pet_lng`/`pet_lat` fields and the `pet_data_history` track table. When the App receives the full sync payload over `testtopic/1`, it performs the coordinate conversion in `processRawData`: calling `CoordTransform.wgs84ToGcj02(lng, lat)` to convert WGS-84 into the GCJ-02 required by AMap.

The conversion function first checks whether the coordinates lie outside mainland China (longitude 73.66–135.05, latitude 3.86–53.55); outside, it returns the original values with no correction. For in-country coordinates the computation splits into three parts: first the offsets relative to the central meridian (105°E, 35°N); then `transformLat` and `transformLng`, two non-linear functions containing sine-series expansions, compute the latitude and longitude offsets; finally the offsets are multiplied by a latitude-based scale factor and added to the original coordinates. The sine-series periodic terms split into short periods (2°, 1°) and long periods (20°, 12°) — the short periods simulate the Mars coordinate system's local deflection ripples, the long periods simulate the geoid model deviation across mainland China as a whole.

The conversion sits in the App rather than Omega because keeping raw WGS-84 values in the database preserves the possibility of switching map vendors later — if the project moves from AMap to Apple Maps or Google Maps (assuming their mainland availability), GCJ-02 offsets no longer apply on high-precision maps. Storing raw values in the database and converting at the interface layer is the precondition for switching map vendors without touching the cloud data chain.

## 5. Full-Chain Low-Power Design

### 5.1 Three-Level Progressive Sleep

The power strategy has three progressive tiers:

1. **Tier one**: the IMU always runs in low-power mode with acceleration-threshold detection completed inside the IMU. When acceleration changes stay below the configured `delta_g` threshold for more than about 2 communication cycles (6 seconds), the system judges the pet stationary and stretches the upload interval from 3 seconds up to 10 s, 30 s, or longer. During long intervals the CPU mostly sits in the WFI (Wait For Interrupt) state.
2. **Tier two**: if the stationary state persists past the system's deep-sleep threshold, the system actively cuts power to the 4G radio and GNSS module — these two remain the main whole-device power consumers even when idle (the 4G module draws about 20–50 mA holding current in RRC IDLE, the GNSS module about 10–20 mA). After power-off, only the IMU's hardware interrupt channel stays alive; the IMU's microamp current is the only continuous draw in deep sleep.
3. **Tier three**: when the IMU detects acceleration or angular velocity exceeding the threshold, it raises an external interrupt to wake the main controller, which restores 4G power, opens a TCP connection, and sends the online notification. From IMU interrupt to device back online, the target latency is under 3 seconds.

### 5.2 Wi-Fi Sniffing & 4G Downgrade Switching

In light-sleep or active states the device's 4G radio is on. Before each report, the device runs a Wi-Fi network scan — if it recognizes a known trusted SSID (home Wi-Fi or a preset hotspot), it immediately powers down the 4G module and transmits over Wi-Fi instead. The logic: Wi-Fi transmission power is far below 4G Cat.1, and pets typically spend most of the day in the home environment. If Wi-Fi can replace 4G indoors, cumulative daily transmission power drops about 60%. The trusted hotspot list is written into device storage via the App's configuration page.

## 6. Device Full-Lifecycle Management

### 6.1 Device Binding & Authentication

Each device is burned at the factory with a unique `device_id` and `product_key` (a PIN anti-counterfeit code, stored in the devices table) in NVRAM. On first use, the user scans the QR code on the device or enters the PIN, calling the PHP endpoint `bind_device.php` to associate the `device_id` with the user account. Immediately after establishing the MQTT connection, the App sends a `sync_request` to the `app/sync/request` topic; Omega's sync handler responds by pulling the complete device state and fence configuration from the database and delivering it to the App over `testtopic/1`.

### 6.2 Track Replay Implementation

Position data is written into `pet_data_history` inside Omega's `on_message`. Each record contains the device ID, longitude/latitude (`decimal(10,6)` precision), battery level, and a timestamp. To support the App's historical track replay, the table carries a `(device_id, created_at)` composite index; the PHP endpoint `get_track.php` returns position-history points filtered by `device_id` and ordered by creation time, and the App draws the track line in chronological order on the AMap WebView.

### 6.3 The Geofence Configuration System

Fence configuration lives in the `pet_config` table, holding the fence center coordinates (`decimal(10,7)` precision) and radius (meters). Every time Omega receives valid GPS data it immediately computes the Haversine distance from the current position to the fence center, setting `is_inside` to 0 when it exceeds the radius. Fence changes have three paths:

1. The App long-presses a point on the map and calls the PHP interface to write the database, with Alpha's 3-second poll detecting the change and pushing it down;
2. Omega's `app/fence/update` handler receives fence coordinates sent directly over MQTT by the App and updates immediately;
3. When the device first comes online, Omega pulls the fence configuration from the database as the default.

Fence-trigger events never produce a push directly — the push layer is handled by `NotificationUtils.ts` judging `isInside` state transitions up front.

## 7. User System & Device-Binding Authentication

### 7.1 Registration & Login Flow

The users table stores identity information — `id`, `username`, `email`, `password_hash`, `token`, `nickname`, `avatar_url`, and `created_at`. Registration requires both a username and an email: the frontend attaches an email verification code to the registration request (generated and sent by `send_code.php`, stored in `verification_codes` with account, code, and expires_at), and after backend validation the password is stored with a password_hash algorithm (PHP's built-in bcrypt wrapper) before being written into users. A token is generated and returned immediately on successful registration — the user is not required to log in again after registering.

Login logic lives in `login.php`. The backend accepts both email and username input: `filter_var` inspects the input format and automatically chooses an email or username query. After finding the user it pulls the bound email address, validates that the code for that email is correct and unexpired, then verifies the password hash with `password_verify()`. Only after all three layers pass is a 16-byte random token generated (`bin2hex(random_bytes(16))`), written into users, and returned to the client. The backend maintains no session; every API call has the PHP side verify login state by SQL-querying the `users.token` field — this token serves as the identity credential for operations like binding devices and switching devices.

### 7.2 Device Binding & the Permission Model

Each hardware device is pre-provisioned in the devices table at the factory with `device_id` (the unique hardware ID) and `product_key` (a security PIN), with the corresponding QR code printed on the shell or manual. `bind_device.php`'s core logic runs in four steps:

1. Validate that the passed-in token is valid;
2. Query devices with the dual condition `device_id` + `product_key` to confirm the device exists in the factory database and the PIN is correct;
3. Query user_devices to confirm the device isn't bound to any user (if already bound to the current user, return "You have already bound it"; if bound to another user, prompt with the masked username to "ask the original owner to unbind it first");
4. Once verified, insert a new record into user_devices and attach the device to the user's account.

The user_devices table is the many-to-many mapping between users and devices — fields include `user_id`, `device_id`, `pet_name`, `is_active`, and `bind_time`. One user can bind multiple devices; one device can bind to only one user. The `is_active` flag marks the user's currently selected active device, so the App opens showing that device's data by default. Switching devices calls `set_active_device.php` to set the new device's `is_active` to 1 while zeroing `is_active` for the user's other devices. `get_devices.php` returns the names, online states, and basic info of all bound devices. `unbind_device.php` deletes the matching record from user_devices by token and `device_id`.

### 7.3 User Profile Management

`update_profile.php` supports changing the nickname, avatar_url, and bio, with the token identifying every modification. `get_profile.php` returns the full user information for the current token. The password reset flow runs in two steps: `send_code.php` first sends a code to the bound email, then `reset_password.php` verifies the code and updates the `password_hash` field in users. The three interfaces (code send → code verify → password update) are independent, with call order controlled by the frontend.

## 8. Smart-Home Linkage

The smart-home module runs independently of the main device-data chain on MQTT port 1884 (the main device-data port is 1883); the two ports' I/O never interfere. The `smart_home_devices` table stores all connected smart devices — fields include `device_id` (identifier, e.g. light, fan, ac), `name` (display name), `type` (device type), `state` (ON/OFF), and `updated_at`. `ha_api.php` exposes four endpoints: `list` (all registered devices), `add` (new device), `delete` (device plus its associated automation rules), and `set` (control a device on/off).

Control commands actually execute by having the PHP side call the system function `exec()` to invoke `/usr/bin/mosquitto_pub`, sending a JSON payload to port 1884 of the MQTT host — the payload format is `{"state": "ON"}` or `{"state": "OFF"}`. After publishing the MQTT command, PHP writes the database and responds to the App directly — a failed MQTT delivery never blocks the App-side operation; it's a fire-and-forget pattern. The linkage between smart devices and pet data is driven by the automation condition engine defined in Omega's `evaluate_automations()`. The engine scans the automations table for `is_active = 1` rules and evaluates each trigger expression of `trigger_type` (ntc_temp / temp / humidity), `operator` (> / <), and threshold. When a condition matches it runs a dedup check — querying `smart_home_devices` to confirm whether the device's current state already equals the target state, executing the MQTT push only if they differ. After firing, it updates the automations table's `last_triggered` timestamp to prevent the same trigger source from re-firing within a short window.

Automation rules are created and managed through the App's `AutomationPage.ets`, which calls the PHP interfaces to write the automations table. A rule's lifecycle is governed by the `is_active` field — the user can activate or deactivate any rule anytime in the App without affecting the others. Deleting a rule makes `ha_api.php` also delete the corresponding `smart_home_devices` record, keeping data consistent. The full firing sequence on each trigger: peripheral report → Omega sensor parsing → `evaluate_automations()` scan → condition match → dedup check → `mosquitto_pub` command → database write → App updates UI after receiving the full sync.

## 9. AI Assistant Integration

The AI assistant is implemented by bridging the DeepSeek V4 Pro large-model API. The App's `PetAiChat.ets` page takes the user's natural-language message, forwards it to the DeepSeek SDK endpoint via HTTP POST, and attaches the device's latest state as context in the request — real-time temperature, steps, active duration, and fence in/out status. The streaming response returned by DeepSeek is chunk-assembled and displayed in the App, forming a chat-style interaction.

The AI bridge lives in the App rather than the cloud to cut latency — routing every message through the full chain HTTP → cloud → DeepSeek → cloud → App would add 500–800 ms of round-trip time. The App connecting straight to DeepSeek's API keeps first-token time within 1–2 seconds. The cloud caches no AI conversation history.
