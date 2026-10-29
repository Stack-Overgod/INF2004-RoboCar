# RoboCar dashboard (local, stdlib only)

```powershell
python -I dashboard\dashboard.py                  # open http://127.0.0.1:8080
python -I dashboard\dashboard.py --start-broker   # also start the broker at launch
```

- **Broker**: Start/Stop buttons run `scripts/mqtt_broker.py` (port 1883, no time limit).
  Each start logs to a new `test/wifi/mqtt_run_<YYYYMMDD_HHMM>.log`. Quitting the
  dashboard (Ctrl+C) stops the broker too. If port 1883 is busy (an old broker still
  running), Start refuses and says so.
- **Commands**: G go, X stop, L/S/R/U = left / straight / right / U-turn (same letters
  as `scripts/send_cmd.py`). Keys: Space stop, G go, arrows for the turns.
- **Telemetry**: everything the broker prints (all `car/01/*` topics + connects).
  Live tilt is not in the firmware: only per-hump max pitch and the motion class.
- The HTTP server binds 127.0.0.1 only; the broker binds 0.0.0.0 so the car can reach it.

## For Claude / troubleshooting (while the dashboard runs)

```sh
curl -s localhost:8080/api/snapshot?n=50   # broker + car link + latest per topic + last 50 events + last 60 s of state
curl -s localhost:8080/api/log?n=200       # last 200 raw broker lines
```

`/?static` renders one snapshot without the live stream (for headless screenshots).

## Car status and commands

- Car pill: **green** = broker has `robocar-01` connected and its last message is under 3 s
  old; **red** otherwise, with the reason (not connected / disconnected / connected but silent).
- Each command waits up to 2 s for the car's `car/01/ack` and says whether it was
  acknowledged, ignored, or got no ack (car not connected or not listening).

## Testing without the car

`python -I dashboard\fake_car.py 20` connects as `robocar-01` from 127.0.0.1,
publishes sample telemetry for 20 s and acks commands like the firmware.
**Don't leave a test dashboard/broker running before a track run**: it holds ports
8080 and 1883, and the real car (which connects to this laptop) would join it.
