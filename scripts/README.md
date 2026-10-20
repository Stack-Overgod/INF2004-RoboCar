# RoboCar local MQTT broker and logger

A small Python broker for receiving the RoboCar's JSON telemetry, saving
messages to a log, and forwarding commands from another MQTT client.

The existing working script is included unchanged. Its known limitations
are documented below.

## 1. Requirements

- Python 3.8 or newer.
- No additional Python packages are required.
- Laptop and RoboCar connected to the same hotspot.
- An MQTT client, such as MQTT Explorer or MQTTX, if sending commands or
  viewing messages through a graphical interface.

Check Python in PowerShell:

```powershell
python --version
```

## 2. Repository layout

```text
INF2004-RoboCar/
├── scripts/
│   ├── mqtt_broker.py
│   └── README.md
└── test/
    └── wifi/
        └── mqtt_run.log
```

The `test` directory contains local working files and is already ignored
by Git. The script and this guide belong in the tracked `scripts` directory.

Run the commands below from the repository root.

## 3. Connect to the hotspot

The team's current setup uses the iPhone hotspot named `Dysteria`.

On the iPhone:

1. Enable Personal Hotspot.
2. Enable Maximise Compatibility for 2.4 GHz compatibility.
3. Keep the Personal Hotspot settings screen open while the devices join.

Connect the laptop to the same hotspot used by the car.

## 4. Match the broker address expected by the car

The broker address is compiled into the firmware from the local
`config/wifi_secrets.h`.

The team's current setup expects:

```c
#define MQTT_BROKER_IP   "172.20.10.3"
#define MQTT_BROKER_PORT 1883
#define MQTT_TOPIC       "car/01/cmd"
```

The laptop running this script must be reachable at that address.

Check the laptop's Wi-Fi IPv4 configuration:

```powershell
ipconfig
```

Do not rely on devices receiving particular addresses based on the order
they join the hotspot.

If the laptop already has `172.20.10.3`, no address change is needed.

If it has a different address, either:

- Assign the expected address manually, after confirming it is unused and
  matches the hotspot's subnet; or
- Update the private firmware configuration to the laptop's actual address,
  rebuild with MQTT enabled, and flash the updated firmware.

For the team's current `172.20.10.0/28` network, the manual settings are:

| Setting | Value |
|---|---|
| IPv4 address | `172.20.10.3` |
| Subnet mask | `255.255.255.240` |
| Prefix length, if requested instead | `28` |
| Gateway | `172.20.10.1` |
| DNS | `172.20.10.1` |

Confirm the actual subnet and gateway before applying these settings to a
different hotspot.

On Windows, manual IPv4 settings are under:

**Settings → Network & internet → Wi-Fi → connected network properties →
IP assignment → Edit → Manual → IPv4**

Restore Automatic (DHCP) when the manual settings are no longer needed.

Keep Wi-Fi passwords and `config/wifi_secrets.h` local.

## 5. Allow the broker through Windows Firewall

The laptop must accept incoming TCP connections on port `1883`.

If Windows displays a firewall prompt when Python starts, allow access for
the hotspot's active network profile.

If necessary, configure an inbound rule for the Python executable and
TCP port `1883`, scoped to the hotspot network. The hotspot may be classified
as a Public network.

## 6. Start the broker

Create the local log directory:

```powershell
New-Item -ItemType Directory -Force test\wifi | Out-Null
```

Start the broker for one hour:

```powershell
python -I scripts\mqtt_broker.py --log test\wifi\mqtt_run.log --secs 3600
```

Expected startup output:

```text
[broker] listening on 0.0.0.0:1883
```

`0.0.0.0` means the broker listens on the laptop's IPv4 network interfaces.
MQTT clients connect using the laptop's actual address, such as
`172.20.10.3`.

### Options

| Option | Meaning |
|---|---|
| `--log PATH` | Append messages to the specified log file |
| `--secs 3600` | Stop after one hour |
| `--port 1883` | Listen on the specified port; the default is 1883 |

The log's parent directory must already exist. Existing logs are appended
to rather than overwritten.

To run until manually stopped:

```powershell
python -I scripts\mqtt_broker.py --log test\wifi\mqtt_run.log
```

Press **Ctrl+C** to stop.

From another PowerShell window, check that something is listening:

```powershell
Get-NetTCPConnection -LocalPort 1883 -State Listen
```

## 7. Power up the car

Start the broker before powering up the car.

Expected output includes lines similar to:

```text
[broker] connection from 172.20.10.2:...
[broker] CONNECT client_id=robocar-01
[broker] SUBSCRIBE car/01/cmd
```

The car's address may differ from the example.

Telemetry should then appear:

- `car/01/state`: approximately four messages per second.
- `car/01/status`: approximately one message every two seconds.
- Event topics when relevant events occur.

Connection and driving-start times vary. Approximately 13 seconds was
observed with a quick connection, but this is not a guaranteed startup time.

The normal MQTT firmware waits for the broker and adds an extra eight-second
delay after a successful connection. It can also begin driving after its
broker-wait timeout without a connection. A `TEST=1` image follows the test
startup sequence instead.

## 8. Read the log

Each entry contains a laptop timestamp, topic and payload:

```text
20:15:46.086 car/01/ack {"rx":"X","cmd":5}
```

Important topics:

| Topic | Contents |
|---|---|
| `car/01/state` | Wheel speeds, motor duties, encoder counts, line bits, distance, motion class, hump peak, forward range and last barcode |
| `car/01/status` | Link state, transport counters, event drops and subsystem liveness |
| `car/01/ack` | Received command character and its decoded command |
| `car/01/barcode` | Decoded barcode |
| `car/01/line` | Line lost or reacquired |
| `car/01/hump` | Completed hump measurement |
| `car/01/impact` | Impact event |
| `car/01/obstacle` | Obstacle profile |
| `car/01/avoid` | Avoidance plan |
| `car/01/move` | Motion completion or abort |
| `car/01/motion` | Motion-class change |

In `state`, `m_l` and `m_r` are commanded motor duties in permille.
Measured wheel speeds are `spd_l` and `spd_r`, in millimetres per second.

Useful `status` fields:

- `tx`: messages accepted by the telemetry transport.
- `fail`: failed transport submissions.
- `drop_fast`, `drop_slow`: dropped events on each event-bus lane.
- `wifi`, `mqtt`: reported network and broker connection state.
- `live`: recent event activity from Buddies 2, 3, 4 and 5, in that order.

A transport acceptance count does not prove that a message reached every
subscriber.

To follow the log in another PowerShell window:

```powershell
Get-Content .\test\wifi\mqtt_run.log -Tail 20 -Wait
```

To hide state, motion and hump messages:

```powershell
Get-Content .\test\wifi\mqtt_run.log -Tail 20 -Wait |
    Where-Object { $_ -notmatch 'car/01/(state|motion|hump)(\s|$)' }
```

## 9. Send commands from another MQTT client

Connect MQTT Explorer, MQTTX or another client using:

| Setting | Value |
|---|---|
| Protocol | MQTT 3.1.1 over TCP |
| Host | Laptop address, currently `172.20.10.3` |
| Port | `1883` |
| TLS | Disabled |
| Username/password | Not required by this helper |

Subscribe to:

```text
car/01/#
```

Publish one ASCII character to:

```text
car/01/cmd
```

| Character | Command |
|---|---|
| `L` or `A` | Turn left |
| `R` or `B` | Turn right |
| `S` or `C` | Go straight |
| `U` or `D` | U-turn |
| `X` or `P` | Stop |

Either case is accepted. Only the first payload byte is interpreted.

For example, publishing `X` should produce an acknowledgement similar to:

```text
car/01/ack {"rx":"X","cmd":5}
```

An acknowledgement reports command decoding. It does not independently
prove that the requested physical movement completed.

Turns and straight commands are accepted while navigation is following
the line or reading a barcode. STOP is handled during a running mission.
Commands do not remotely restart a stopped mission.

The existing firmware's reconnect backoff can delay command handling,
including STOP, by up to eight seconds.

## 10. Troubleshooting

| Symptom | Check |
|---|---|
| `python` is not recognised | Python installation and PATH |
| Log file cannot be opened | Create `test\wifi` and check write permissions |
| Port 1883 is already in use | Another broker or previous script instance may be running |
| Broker listens but the car never connects | Hotspot connection, firewall, laptop IPv4 address and the firmware's compiled broker address |
| Connection appears but no telemetry follows | Firmware MQTT configuration and telemetry startup |
| Client cannot connect | Use MQTT 3.1.1 over TCP, port 1883, with TLS disabled |
| Command is acknowledged but no movement occurs | Navigation state; acknowledgement only confirms decoding |
| S1-W09 fails while external commands work | Publisher self-delivery limitation described below |

## 11. Known limitations of the unchanged script

This helper implements a limited MQTT 3.1.1 subset for local development.

### Publisher self-delivery

The script does not forward a publication back to the same connection that
published it, even when that connection has a matching subscription.

External-client commands can still reach the car because the client and
car use separate connections.

The firmware's **S1-W09** test intentionally publishes a command from the car
to its own subscribed command topic. This helper suppresses that return
delivery, so the unchanged script is unsuitable for validating S1-W09.

### Unsubscribe

The script acknowledges UNSUBSCRIBE packets but does not remove the
requested subscription filters. A client may continue receiving messages
until it disconnects.

### Future binary telemetry

Forwarding preserves payload bytes, but logging decodes them as UTF-8 text
with replacement characters.

Future CR1 binary telemetry will need a reversible log representation,
such as hexadecimal or Base64, to preserve all payload bytes.

### Other unsupported features

The helper does not implement:

- Retained messages.
- Persistent sessions.
- Last Will messages.
- Authentication or TLS.
- MQTT 5.
- QoS 2.

Forwarded publications use QoS 0.

## 12. Test evidence

Routine working logs belong under the ignored `test/wifi` directory.

When a run is used as formal Buddy One evidence, save the selected log under
`testing/subsystem1/logs/` and record its configuration, firmware commit,
procedure and result in `testing/subsystem1/TEST_LOG.md`.

Do not count copying this script into the repository as a new passing test.

For the firmware's topic and payload reference, see
[MQTT documentation](../docs/buddy1-telemetry/MQTT.md).
