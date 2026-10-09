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
* **Anti-hunting hysteresis** (`spin_up_margin`, default 48): while a fan
  sits at its baseline duty (where it may stall), small temp bumps do NOT
  restart it - it only spins up when the target exceeds `min_pwm +
  spin_up_margin`. This kills the stall -> slight warmup -> restart -> cool
  down -> stall limit cycle.
* First cycle after boot/restart: an upward target is applied immediately
  (never start under-cooling a hot GPU); downward changes ramp down slowly.
* Fail-safe: if no temperature can be read - **including the GPU being
  unplugged or the amdgpu driver not loaded** - all fans go to
  `on_sensor_error_pwm` (default: full speed). The service does NOT crash:
  it logs one error, keeps polling for the GPU and resumes normal control
  automatically when it appears again (e.g. after `modprobe amdgpu`).
* **Self-healing pwm writes**: the intended duty is re-applied to every fan
  on *every* cycle (not write-once), so external writers that clobber sysfs
  (`sudo pwmconfig`, a stray `echo 0 > pwmN`, ...) are undone within one
  `interval` instead of silently winning until the next restart.
* Optional rpm monitoring (`fan_rpm_min`): warns in the journal when a fan
  appears stuck/dead (3 consecutive low readings) — the warning includes the
  current GPU temps. If a `--calibrate` fit exists, the threshold becomes
  `max(fan_rpm_min, 50% of expected rpm at the current pwm)`, so a motor
  stuck at half speed is caught even far above the absolute minimum.
* **Synced-group desync warning** (`sync_warn_pct`, `sync_warn_min_diff`):
  fans on one pwm net should spin at (nearly) the same speed; a spread of
  more than max(min_diff, pct% of the higher rpm) sustained for 3 cycles
  raises a WARNING with both rpm's (and logs recovery). Normal bearing
  spread is a few percent — the defaults (15% / 150 rpm) don't false-alarm.
* **Stall control** (`allow_stall`, opt-in): the fan may stop completely
  (pwm 0) when the GPU is cool. A temperature hysteresis band keeps it from
  flapping on oscillating temps: it stalls only below `stall_temp` (and only
  after `stall_confirm` cycles), and after a stall it restarts only above
  `stall_temp + stall_start_margin` — the card may warm up a bit while
  uncooled, that's fine. If it keeps chasing (≥ `stall_chase_limit`
  stall/run flips, counted per run - no time window, since a chase cycle
  can take many minutes while the card slowly re-heats) it gives up on
  stalling for the run and holds the fan at min rpm. Running fans never go
  below their `min_pwm`; 0 is only the explicit stalled state.
* A periodic **status line** (`status_every`, default every 6th cycle ≈ 30 s)
  logs fan state, temperatures and target/dominant sensor, so the journal
  always shows what is happening even when nothing changes. Change lines use
  the same section order (FAN | TEMPS | CONTROL), e.g.

      status: duct:pwm=72 rpm=1004/1092 | gpu0[edge=28.0 junction=29.0 mem=26.0] | target=32 dominant=gpu0/edge
      [duct] pwm 72 -> 136 | gpu0[edge=55.0 junction=75.0 mem=57.0] | target=136 dominant=gpu0/junction

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

## Calibrating the pwm→rpm curve (`--calibrate`)

```bash
sudo systemctl stop fanctrl                      # it would fight the sweep
sudo python3 /usr/local/bin/fanctrl.py --calibrate [--cal-points 6] [--cal-hold 4]
sudo systemctl start fanctrl
```

Sweeps each fan group through several pwm steps (default 6, from the group's
min to max), holds each for `--cal-hold` seconds (default 4) and reads every
member's rpm. Per fan a least-squares line `rpm ~= a*pwm + b` is fitted and
saved to `calibration_file` (default `/var/lib/fanctrl/calibration.json`,
merged with existing entries); original pwm/enable values are restored.
The controller loads the fit at startup and uses it for the dead-fan
threshold (see above). Fans spin up/down during the sweep - don't run it on
CPU fans blindly.

## Discovering fans (`--detect` / `--probe`)

```bash
sudo python3 /usr/local/bin/fanctrl.py --detect
# read-only inventory of every hwmon pwm/fan channel + suggested [fanN] config

sudo python3 /usr/local/bin/fanctrl.py --probe it8728:pwm2
# active test: spins the given pwm up/down briefly and reports which fan's
# rpm reacts (original values are restored). 'all' probes every pwm with an
# enable file. Fans spin up for a few seconds - don't probe CPU fans blindly.
```

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
sudo python3 /usr/local/bin/fanctrl.py --detect    # find pwm/fan channels
sensors | grep -A3 it8728              # pwm2 % + fan2 rpm
```

## Tuning (in `/etc/fanctrl.conf`)

* `interval` — polling period; higher = calmer but slower reaction.
* `ramp_up_step` / `ramp_down_step` — max pwm change per cycle.
* `spin_up_margin` — anti-hunting hysteresis above the baseline (see above).
* `[fanN] min_pwm` — baseline duty; running fans never go below it. With
  `allow_stall = true` the fan may also sit at pwm 0 when cool (see below).
* `[gpuN] *_points = 45:32, 60:192, 70:255` (°C:pwm pairs, linear between
  points, clamped at both ends).
* `allow_stall`, `stall_temp`, `stall_start_margin`, `stall_confirm`,
  `stall_chase_limit` — stall control (see above).
* `sync_warn_pct` / `sync_warn_min_diff` — desync warning for synced groups.
* `fan_rpm_min` — dead-fan threshold (calibration-aware if a fit exists).
* `status_every` — status line every N cycles (0 disables).

After editing: `sudo systemctl restart fanctrl`.

## Safety notes

* If the service is stopped or crashes, the hardware keeps its last pwm value
  (the fan was running) — no sudden stop of cooling.
* The controller never writes below a fan's `min_pwm` and only touches the
  configured pwm/enable files.
* A dead fan shows up in the journal as "fan rpm ... below threshold" with the
  current temps attached; the periodic status lines keep showing temps rising.
