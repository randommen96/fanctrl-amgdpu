# fanctrl

Chassis-fan controller driven by amdgpu GPU temperatures.

Cools passively-cooled GPU(s) via chassis fan(s) on motherboard PWM headers
(it8728 super-IO, `hwmon2/pwm2` in the default config). Reads the same edge /
junction / mem temperatures that `amdgpu_top` and `sensors` show, straight
from sysfs:

    /sys/class/drm/cardN/device/hwmon/<hwmonM>/temp{1,2,3}_input  (millidegrees)

## How it works

* Every `interval` seconds (default 5) it reads all enabled GPU sensors.
* Each sensor has its own piecewise-linear `temp_c:pwm` curve in the config.
* Every fan is driven at the **maximum** of all (GPU, sensor) targets, so
  *any* sensor on *any* GPU exceeding its limits bumps the speed
  (e.g. junction >= 70 °C -> 100 %).
* Ramp limiting: fast ramp-up (`ramp_up_step`, default 64/cycle) for safety,
  slow ramp-down (`ramp_down_step`, default 8/cycle) to avoid hunting.
* First cycle after boot/restart: an upward target is applied immediately
  (never start under-cooling a hot GPU); downward changes ramp down slowly.
* Fail-safe: if no temperature can be read, all fans go to
  `on_sensor_error_pwm` (default: full speed) and an error is logged.
* Optional rpm monitoring (`fan_rpm_min`): warns in the journal when a fan
  appears stuck/dead (3 consecutive low readings) — the warning includes the
  current GPU temps.
* A periodic **status line** (`status_every`, default every 6th cycle ≈ 30 s)
  logs all temperatures, target pwm and measured rpm, so the journal always
  shows what is happening even when nothing changes.

## Multiple GPUs and multiple fans

```ini
[fan1]
pwm_path = /sys/class/hwmon/hwmon2/pwm2
...
group    = duct        # <- sync group name (empty = individual)

[fan2]
pwm_path = /sys/class/hwmon/hwmon3/pwm1
...
group    = duct

[gpu0]
junction_points = 45:32, 60:192, 70:255

[gpu1]
hwmon           = /sys/class/hwmon/hwmon5   # optional pin; auto-assigned otherwise
junction_points = 45:32, 60:192, 70:255
```

* One `[fanN]` section per fan — each with its own `pwm_path`, `enable_path`,
  `rpm_path` and its own **`min_pwm` / `max_pwm`** (yes, you can set a minimum
  pwm per fan; default 32 ≈ 12 %).
* One `[gpuN]` section per GPU — hwmon dirs are auto-assigned in order
  (card0 → gpu0, card1 → gpu1, ...); pin one with `hwmon = ...`. A sensor
  curve is only active when its `*_points` line exists.
* **Sync groups**: fans with the same non-empty `group` are driven together —
  one shared target/pwm is computed and written identically to all members
  (e.g. a fan duct where both fans must spin at the same speed). At startup a
  group starts at the *highest* member pwm so nobody is dropped. RPM
  monitoring stays per-fan, so a dead motor in a synced pair still raises its
  own warning.

The old single-fan/single-GPU format (paths in `[main]`, `[edge]`/`[junction]`/
`[mem]` sections) keeps working.

## Files

| file | purpose |
|---|---|
| `/usr/local/bin/fanctrl.py` | the controller (Python 3, stdlib only) |
| `/etc/fanctrl.conf` | config: fans, GPUs, curves, ramps, status interval |
| `/etc/systemd/system/fanctrl.service` | systemd unit, starts at boot, restarts on crash |

## Usage

```bash
sudo systemctl status fanctrl          # service state
journalctl -u fanctrl -f               # live log (pwm changes, status lines, warnings)
sudo python3 /usr/local/bin/fanctrl.py --once      # single control cycle
sudo python3 /usr/local/bin/fanctrl.py --dry-run   # show decisions, no writes
sensors | grep -A3 it8728              # pwm2 % + fan2 rpm
```

## Tuning (in `/etc/fanctrl.conf`)

* `interval` — polling period; higher = calmer but slower reaction.
* `ramp_up_step` / `ramp_down_step` — max pwm change per cycle.
* `[fanN] min_pwm` — baseline duty (~12 %); keeps the fan spinning for airflow.
* `[gpuN] *_points = 45:32, 60:192, 70:255` (°C:pwm pairs, linear between
  points, clamped at both ends).
* `status_every` — status line every N cycles (0 disables).

After editing: `sudo systemctl restart fanctrl`.

## Safety notes

* If the service is stopped or crashes, the hardware keeps its last pwm value
  (the fan was running) — no sudden stop of cooling.
* The controller never writes below a fan's `min_pwm` and only touches the
  configured pwm/enable files.
* A dead fan shows up in the journal as "fan rpm ... below threshold" with the
  current temps attached; the periodic status lines keep showing temps rising.
