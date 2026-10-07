#!/usr/bin/env python3
"""
fanctrl - control chassis fan(s) (PWM headers) from GPU temperature(s).

Cools passively-cooled GPU(s) via chassis fan(s) on motherboard PWM headers.
Reads amdgpu sensor temperatures (edge / junction / mem - the same values
shown by `amdgpu_top` and `sensors`) from sysfs hwmon, maps each enabled
sensor through its own piecewise-linear temperature->PWM curve, and drives
each fan at the MAXIMUM of all (GPU, sensor) targets, so that any single
sensor on any GPU exceeding its limits bumps every fan's speed.

Features:
  * multiple GPUs ([gpu0], [gpu1], ...) and multiple fans ([fan1], [fan2], ...)
  * synced fan groups (same 'group' -> one shared pwm, e.g. a fan duct)
  * anti-hunting hysteresis around the idle point (spin_up_margin): while the
    fan sits at its baseline duty it is NOT restarted by small temp bumps -
    it only spins up when there is real cooling demand
  * fast ramp-up / slow ramp-down per cycle; immediate apply on first cycle
  * GPU absent (unplugged or driver not loaded) -> fail-safe mode: fans are
    driven to on_sensor_error_pwm and the service keeps polling for the GPU,
    resuming normal control when it appears again (no crash-loop)
  * per-fan rpm monitoring (dead-fan warning includes current temps)
  * periodic status line so the journal always shows what is happening
  * --detect / --probe: discover which pwm channels control which fans

Usage:
  fanctrl.py [--config /etc/fanctrl.conf] [--once] [--dry-run]
  fanctrl.py --detect
  fanctrl.py --probe it8728:pwm2[,hwmon3:pwm1,...]   (or --probe all)

Stdlib only. Designed to run under systemd (logs to stderr -> journal).
"""

import argparse
import glob
import logging
import os
import re
import signal
import sys
import time

log = logging.getLogger("fanctrl")

SENSORS = ("edge", "junction", "mem")


# --------------------------------------------------------------------------
# config
# --------------------------------------------------------------------------

def parse_points(spec, where):
    """Parse '45:32, 60:192, 70:255' -> [(45.0, 32), (60.0, 192), ...]"""
    points = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            t, p = part.split(":")
            points.append((float(t), int(p)))
        except ValueError:
            sys.exit(f"fanctrl: {where}: bad point '{part}', expected 'temp_c:pwm'")
    if len(points) < 2:
        sys.exit(f"fanctrl: {where}: needs at least 2 points, got {len(points)}")
    points.sort(key=lambda x: x[0])
    for a, b in zip(points, points[1:]):
        if a[0] == b[0]:
            sys.exit(f"fanctrl: {where}: duplicate temps at {a[0]}")
    return points


class FanConfig:
    def __init__(self, name, pwm_path, enable_path, rpm_path, min_pwm, max_pwm, group=""):
        self.name = name
        self.pwm_path = pwm_path
        self.enable_path = enable_path
        self.rpm_path = rpm_path
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        self.group = group  # fans sharing a group are driven at the same pwm


class GpuConfig:
    def __init__(self, name, hwmon_dir, curves):
        self.name = name
        self.hwmon_dir = hwmon_dir      # None -> auto-assign in order
        self.curves = curves            # {sensor: [(temp_c, pwm), ...]}


class Config:
    def __init__(self, path):
        import configparser
        cp = configparser.ConfigParser(interpolation=None,
                                       inline_comment_prefixes=("#", ";"))
        try:
            with open(path) as f:
                cp.read_file(f)
        except OSError as e:
            sys.exit(f"fanctrl: cannot read config {path}: {e}")

        m = cp["main"]
        self.interval = float(m.get("interval", "5"))
        self.ramp_up_step = int(m.get("ramp_up_step", "64"))
        self.ramp_down_step = int(m.get("ramp_down_step", "8"))
        # anti-hunting: while a fan sits at its baseline (min_pwm), it is only
        # spun up when the target exceeds min_pwm + spin_up_margin. Small temp
        # bumps that would otherwise restart a stalled fan are ignored.
        self.spin_up_margin = int(m.get("spin_up_margin", "48"))
        self.on_sensor_error_pwm = int(m.get("on_sensor_error_pwm", "255"))
        self.fan_rpm_min = int(m.get("fan_rpm_min", "0"))      # 0 = disabled
        self.status_every = int(m.get("status_every", "6"))    # 0 = disabled
        self.log_level = m.get("log_level", "info").upper()

        # ---- fans: [fan1], [fan2], ... -------------------------------------
        self.fans = []
        fan_sections = {s for s in cp.sections() if re.fullmatch(r"fan\d+", s)}
        if not fan_sections:
            sys.exit("fanctrl: no fan configured (add a [fan1] section with pwm_path)")
        for name in sorted(fan_sections, key=lambda s: int(s[3:])):
            s = cp[name]
            pwm_path = s.get("pwm_path", "")
            if not pwm_path:
                sys.exit(f"fanctrl: [{name}] needs pwm_path")
            self.fans.append(FanConfig(
                name=name,
                pwm_path=pwm_path,
                enable_path=s.get("enable_path", ""),
                rpm_path=s.get("rpm_path", ""),
                min_pwm=int(s.get("min_pwm", "32")),
                max_pwm=int(s.get("max_pwm", "255")),
                group=s.get("group", "").strip(),
            ))

        for f in self.fans:
            if not (0 <= f.min_pwm <= f.max_pwm <= 255):
                sys.exit(f"fanctrl: [{f.name}] bad pwm range {f.min_pwm}..{f.max_pwm}")

        # ---- GPUs: [gpu0], [gpu1], ... --------------------------------------
        self.gpus = []
        gpu_sections = {s for s in cp.sections() if re.fullmatch(r"gpu\d*", s)}
        if not gpu_sections:
            sys.exit("fanctrl: no GPU configured (add a [gpu0] section with *_points)")
        for name in sorted(gpu_sections, key=lambda s: int(s[3:] or 0)):
            s = cp[name]
            if s.get("enabled", "true").strip().lower() in ("0", "false", "no"):
                continue
            curves = {}
            for sensor in SENSORS:
                spec = s.get(f"{sensor}_points", "")
                if spec:
                    curves[sensor] = parse_points(spec, f"[{name}] {sensor}")
            if not curves:
                sys.exit(f"fanctrl: [{name}] has no *_points configured")
            self.gpus.append(GpuConfig(name, s.get("hwmon", "") or None, curves))

        if not self.gpus:
            sys.exit("fanctrl: no enabled GPU configured")


def curve_value(points, temp_c):
    """Piecewise-linear mapping temp(°C) -> pwm for sorted (temp, pwm) points."""
    if temp_c <= points[0][0]:
        return points[0][1]
    if temp_c >= points[-1][0]:
        return points[-1][1]
    for (t0, p0), (t1, p1) in zip(points, points[1:]):
        if t0 <= temp_c <= t1:
            frac = (temp_c - t0) / (t1 - t0)
            return int(round(p0 + frac * (p1 - p0)))


# --------------------------------------------------------------------------
# sysfs helpers
# --------------------------------------------------------------------------

def find_amdgpu_hwmons():
    """All amdgpu hwmon directories, sorted (one per GPU)."""
    candidates = []
    for dev in sorted(glob.glob("/sys/class/drm/card*/device/hwmon/*")):
        try:
            if open(os.path.join(dev, "name")).read().strip() == "amdgpu":
                candidates.append(dev)
        except OSError:
            pass
    if not candidates:  # fallback: any hwmon named amdgpu
        for dev in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
            try:
                if open(os.path.join(dev, "name")).read().strip() == "amdgpu":
                    candidates.append(dev)
            except OSError:
                pass
    return candidates


def build_temp_map(hwmon_dir):
    """Map sensor name (edge/junction/mem) -> sysfs file, using *_label files."""
    label_files = {}
    for f in glob.glob(os.path.join(hwmon_dir, "temp*_input")):
        base = os.path.basename(f)[: -len("_input")]  # e.g. temp2
        try:
            label = open(os.path.join(hwmon_dir, base + "_label")).read().strip()
        except OSError:
            label = ""
        if label:
            label_files[label.lower()] = f
    fallback = {"edge": "temp1_input", "junction": "temp2_input", "mem": "temp3_input"}
    m = {}
    for name in SENSORS:
        if name in label_files:
            m[name] = label_files[name]
        else:
            f = os.path.join(hwmon_dir, fallback[name])
            if os.path.exists(f):
                m[name] = f
    return m


def read_temp_c(path):
    """Read a millidegree sysfs temp file -> °C float. Raises on failure."""
    with open(path) as f:
        raw = f.read().strip()
    v = int(raw)
    if v <= -20000 or v > 200000:  # sanity: off / bogus reading
        raise ValueError(f"implausible temp {v} mdeg from {path}")
    return v / 1000.0


def read_sysfs_int(path, default=None):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return default


def write_sysfs(path, value):
    with open(path, "w") as f:
        f.write(str(value))


# --------------------------------------------------------------------------
# controller
# --------------------------------------------------------------------------

class FanGroup:
    """Runtime state of one or more fans driven together.

    Fans with the same non-empty 'group' config key form one group: a single
    target/pwm is computed and written identically to every member (e.g. a
    fan duct where both fans must spin at the same speed). Members without a
    group are controlled individually. RPM monitoring stays per-fan so a dead
    motor in a synced pair still raises its own warning.
    """

    def __init__(self, name, members):
        self.name = name
        self.members = members  # list[FanConfig]
        pws = []                # start at the HIGHEST member pwm (never drop anyone)
        for m in members:
            p = read_sysfs_int(m.pwm_path, m.min_pwm)
            pws.append(max(0, min(255, p if p is not None else m.min_pwm)))
        self.current_pwm = max(pws) if pws else 0
        self.actual = {m.name: p for m, p in zip(members, pws)}
        self.base = max(m.min_pwm for m in members)  # baseline duty (fan may stall here)
        self.first_cycle = True
        self.rpm_state = {m.name: {"streak": 0, "warned": False} for m in members}

    def set_pwm(self, value, dry_run):
        """Write the same pwm to every member (clamped by each member's min/max)."""
        changed = False
        for m in self.members:
            v = max(m.min_pwm, min(m.max_pwm, value))
            if v == self.actual[m.name]:
                continue
            if not dry_run:
                try:
                    if m.enable_path:
                        write_sysfs(m.enable_path, 1)  # manual/direct mode
                    write_sysfs(m.pwm_path, v)
                except OSError as e:
                    log.error("[%s] writing pwm %d failed: %s", m.name, v, e)
                    continue
            else:
                log.info("[dry-run] %s would set pwm %d (currently %d)",
                         m.name, v, self.actual[m.name])
            self.actual[m.name] = v
            changed = True
        return changed


class FanController:
    def __init__(self, cfg, dry_run=False):
        self.cfg = cfg
        self.dry_run = dry_run

        self.gpus = []
        self.gpu_loss_logged = False
        self._setup_gpus(log_new=True)

        # build fan groups: same non-empty group name -> driven together
        by_group, order = {}, []
        for f in cfg.fans:
            key = f.group if f.group else f.name
            if key not in by_group:
                by_group[key] = []
                order.append(key)
            by_group[key].append(f)
        self.groups = [FanGroup(k, by_group[k]) for k in order]
        for g in self.groups:
            who = "+".join(m.name for m in g.members)
            log.info("[%s] current pwm at startup: %d (baseline %d, members: %s)",
                     g.name, g.current_pwm, g.base, who)

        self.consec_errors = 0
        self.status_counter = 0
        self.last_temps_str = ""
        self._stop = False

    # ---- GPU setup / rescan --------------------------------------------------

    def _setup_gpus(self, log_new=False):
        """(Re)build self.gpus from config + currently present amdgpu hwmons."""
        new_gpus = []
        auto_pool = list(find_amdgpu_hwmons())
        for gpu in self.cfg.gpus:
            if gpu.hwmon_dir:
                if not os.path.isdir(gpu.hwmon_dir):
                    continue  # this GPU is absent right now
            else:
                if not auto_pool:
                    continue
                gpu.hwmon_dir = auto_pool.pop(0)
            temp_map = build_temp_map(gpu.hwmon_dir)
            sensors = {n: p for n, p in temp_map.items() if n in gpu.curves}
            missing = [n for n in gpu.curves if n not in temp_map]
            if missing and log_new:
                log.warning("[%s] no sysfs sensor(s) for: %s (skipped)",
                            gpu.name, ", ".join(missing))
            if sensors:
                new_gpus.append((gpu, sensors))
                if log_new:
                    for name, path in sorted(sensors.items()):
                        log.info("[%s] %-9s -> %s (hwmon: %s)",
                                 gpu.name, name, path, gpu.hwmon_dir)
        self.gpus = new_gpus

    # ---- reading -----------------------------------------------------------

    def read_temps(self):
        """Return {gpu_name: {sensor: temp_c}} for all readable sensors."""
        temps = {}
        for gpu, sensors in self.gpus:
            ok = {}
            for name, path in sensors.items():
                try:
                    ok[name] = read_temp_c(path)
                except (OSError, ValueError) as e:
                    log.debug("[%s/%s] read failed: %s", gpu.name, name, e)
            if ok:
                temps[gpu.name] = ok
        return temps

    @staticmethod
    def fmt_temps(temps):
        parts = []
        for gpu_name in sorted(temps):
            s = " ".join(f"{n}={t:.1f}" for n, t in sorted(temps[gpu_name].items()))
            parts.append(f"{gpu_name}[{s}]")
        return " ".join(parts)

    def target_for_fan(self, temps):
        """Max over all (gpu, sensor) curves. Returns (pwm, 'gpu/sensor' or None)."""
        best, dom = 0, None
        for gpu_name in sorted(temps):
            gpu = next(g for g, _ in self.gpus if g.name == gpu_name)
            for sensor, t in temps[gpu_name].items():
                p = curve_value(gpu.curves[sensor], t)
                if p > best:
                    best, dom = p, f"{gpu_name}/{sensor}"
        return best, dom

    # ---- one control cycle ---------------------------------------------------

    def cycle(self):
        # no GPU present at all (unplugged / driver not loaded)? keep polling
        if not self.gpus:
            self._setup_gpus(log_new=True)
            if not self.gpus:
                self.consec_errors += 1
                fail_pwm = self.cfg.on_sensor_error_pwm
                if not self.gpu_loss_logged:
                    log.error("no amdgpu GPU/hwmon present - driving all fans to "
                              "fail-safe pwm %d and polling for the GPU to return",
                              fail_pwm)
                    self.gpu_loss_logged = True
                else:
                    log.debug("still no amdgpu GPU (consec=%d)", self.consec_errors)
                for g in self.groups:
                    g.set_pwm(fail_pwm, self.dry_run)
                    g.current_pwm = fail_pwm
                return

        if self.gpu_loss_logged:
            log.info("amdgpu GPU detected again - resuming normal control")
            self.gpu_loss_logged = False

        temps = self.read_temps()
        tstr = self.fmt_temps(temps) or "no temps readable"
        self.last_temps_str = tstr

        if not temps:
            self.consec_errors += 1
            fail_pwm = self.cfg.on_sensor_error_pwm
            log.error("no temperature readable (%d consecutive), "
                      "setting all fans to fail-safe pwm %d", self.consec_errors, fail_pwm)
            for g in self.groups:
                g.set_pwm(fail_pwm, self.dry_run)
                g.current_pwm = fail_pwm
            return

        self.consec_errors = 0
        target, dom = self.target_for_fan(temps)

        # ramp limiting + anti-hunting hysteresis:
        #  - fast up (safety), slow down (no hunting)
        #  - while a fan sits at its baseline duty it is NOT restarted by small
        #    temp bumps; it only spins up when target >= base + spin_up_margin
        #  - first cycle after boot/restart applies an upward target immediately
        #    (never start under-cooling a hot GPU)
        for g in self.groups:
            if g.current_pwm <= g.base:
                if target >= g.base + self.cfg.spin_up_margin:
                    new = target if g.first_cycle else \
                        min(target, g.current_pwm + self.cfg.ramp_up_step)
                else:
                    new = g.base  # hold low - no chasing / stall-restart flapping
            elif target > g.current_pwm:
                new = min(target, g.current_pwm + self.cfg.ramp_up_step)
            elif target < g.current_pwm:
                new = max(target, g.current_pwm - self.cfg.ramp_down_step)
            else:
                new = target
            g.first_cycle = False

            if new != g.current_pwm:
                log.info("[%s] pwm %d -> %d (target %d, dominant: %s; %s)",
                         g.name, g.current_pwm, new, target, dom, tstr)
                g.set_pwm(new, self.dry_run)
            g.current_pwm = new

        for g in self.groups:
            for m in g.members:
                self.check_fan_rpm(g, m)

        # periodic status line so the journal always shows activity
        self.status_counter += 1
        if self.cfg.status_every > 0 and self.status_counter % self.cfg.status_every == 0:
            fans = " ".join(
                f"{g.name}:pwm={g.current_pwm}" +
                (" rpm=" + "/".join(str(self._rpm(m)) for m in g.members)
                 if any(m.rpm_path for m in g.members) else "")
                for g in self.groups)
            log.info("status: %s | target=%d dominant=%s | %s", tstr, target, dom, fans)

    def _rpm(self, m):
        return read_sysfs_int(m.rpm_path)

    def check_fan_rpm(self, g, m):
        """Per-fan rpm monitoring (even inside a synced group)."""
        if not m.rpm_path or self.cfg.fan_rpm_min <= 0:
            return
        st = g.rpm_state[m.name]
        rpm = self._rpm(m)
        if rpm is None:
            return
        if g.current_pwm >= m.min_pwm and rpm < self.cfg.fan_rpm_min:
            st["streak"] += 1
            if st["streak"] == 3 and not st["warned"]:
                log.warning("[%s] fan rpm %d below threshold %d while pwm=%d - "
                            "fan may be stuck or dead | %s",
                            m.name, rpm, self.cfg.fan_rpm_min, g.current_pwm,
                            self.last_temps_str)
                st["warned"] = True
        else:
            if st["warned"]:
                log.info("[%s] fan rpm recovered: %d | %s", m.name, rpm, self.last_temps_str)
            st["streak"] = 0
            st["warned"] = False

    # ---- main loop ---------------------------------------------------------

    def run(self, once=False):
        signal.signal(signal.SIGTERM, self._handle_stop)
        signal.signal(signal.SIGINT, self._handle_stop)
        log.info("fanctrl starting: fans=%s gpus=%s interval=%.1fs "
                 "ramp_up=%d ramp_down=%d spin_up_margin=%d dry_run=%s",
                 ",".join(g.name for g in self.groups),
                 ",".join(g.name for g, _ in self.gpus) or "<none present>",
                 self.cfg.interval, self.cfg.ramp_up_step, self.cfg.ramp_down_step,
                 self.cfg.spin_up_margin, self.dry_run)
        while not self._stop:
            started = time.monotonic()
            try:
                self.cycle()
            except Exception as e:  # never die on a transient error
                log.exception("cycle failed: %s", e)
            if once:
                break
            deadline = started + self.cfg.interval
            while not self._stop and time.monotonic() < deadline:
                time.sleep(0.2)
        log.info("fanctrl stopped (last pwm: %s)",
                 ", ".join(f"{g.name}={g.current_pwm}" for g in self.groups))

    def _handle_stop(self, signum, frame):
        log.info("received signal %d", signum)
        self._stop = True


# --------------------------------------------------------------------------
# fan discovery: --detect (read-only) and --probe (active)
# --------------------------------------------------------------------------

def scan_hwmon_channels():
    """Inventory of all hwmon pwm/fan channels. Returns list of dicts."""
    chans = []
    for dev in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
        try:
            name = open(os.path.join(dev, "name")).read().strip()
        except OSError:
            continue
        entry = {"hwmon": os.path.basename(dev), "name": name, "pwms": [], "fans": []}
        for f in sorted(glob.glob(os.path.join(dev, "pwm[0-9]*"))):
            b = os.path.basename(f)
            if not re.fullmatch(r"pwm\d+", b):
                continue
            n = int(b[3:])
            entry["pwms"].append({
                "num": n, "path": f,
                "enable": os.path.join(dev, f"pwm{n}_enable"),
                "value": read_sysfs_int(f),
                "min": read_sysfs_int(os.path.join(dev, f"pwm{n}_min")),
                "max": read_sysfs_int(os.path.join(dev, f"pwm{n}_max"))})
        for f in sorted(glob.glob(os.path.join(dev, "fan[0-9]*_input"))):
            b = os.path.basename(f)[: -len("_input")]
            n = int(b[3:])
            entry["fans"].append({"num": n, "path": f,
                                  "rpm": read_sysfs_int(f),
                                  "min": read_sysfs_int(os.path.join(dev, f"fan{n}_min"))})
        if entry["pwms"] or entry["fans"]:
            chans.append(entry)
    return chans


def cmd_detect():
    """Read-only: list pwm/fan channels and suggest a config."""
    print("hwmon pwm/fan inventory (read-only):")
    suggested = []
    for e in scan_hwmon_channels():
        print(f"\n{e['hwmon']} ({e['name']})")
        for p in e["pwms"]:
            mode = ""
            if os.path.exists(p["enable"]):
                ev = read_sysfs_int(p["enable"])
                mode = " [manual]" if ev else " [auto]"
            print(f"  pwm{p['num']}: value={p['value']} min={p['min']} max={p['max']}{mode}")
        for f in e["fans"]:
            guess = "  <- likely pwm%d" % f["num"] if any(p["num"] == f["num"] for p in e["pwms"]) else ""
            print(f"  fan{f['num']}: {f['rpm']} rpm (min={f['min']}){guess}")
        for p in e["pwms"]:
            if any(f["num"] == p["num"] for f in e["fans"]) and os.path.exists(p["enable"]):
                suggested.append((e, p))
    if suggested:
        print("\nsuggested config (verify with --probe before using!):")
        for i, (e, p) in enumerate(suggested, 1):
            dev = os.path.dirname(p["path"])
            print(f"""
[fan{i}]
pwm_path    = {p['path']}
enable_path = {p['enable']}
rpm_path    = {os.path.join(dev, f"fan{p['num']}_input")}
min_pwm     = 32
max_pwm     = 255
group       =""")
    else:
        print("\nno pwm channels with a matching fan input found")


def cmd_probe(specs, hold):
    """Active test: nudge each given pwm and see which fan reacts.

    specs: 'chip:pwmN' (chip = driver name like it8728 or dir like hwmon2)
           or ['all'] for every pwm that has an enable file.
    Original pwm/enable values are restored after each probe.
    """
    want_all = "all" in specs
    targets = []
    for e in scan_hwmon_channels():
        for p in e["pwms"]:
            if not os.path.exists(p["enable"]):
                continue
            keys = {f"{e['name']}:pwm{p['num']}", f"{e['hwmon']}:pwm{p['num']}"}
            if want_all or keys & set(specs):
                targets.append((e, p))
    if not targets:
        sys.exit(f"fanctrl: no probeable pwm matching {specs} (see --detect)")

    print(f"probing {len(targets)} channel(s), hold {hold:g}s each - fans spin up "
          "briefly. Ctrl-C to abort.")
    time.sleep(2)

    results = []
    for e, p in targets:
        n = p["num"]
        pwm_path, en_path = p["path"], p["enable"]
        orig_pwm = read_sysfs_int(pwm_path, 0) or 0
        orig_en = read_sysfs_int(en_path, 0) or 0
        fans = {f["num"]: f["path"] for f in e["fans"]}
        try:
            write_sysfs(en_path, 1)
            write_sysfs(pwm_path, 255)
            time.sleep(hold)
            high = {k: read_sysfs_int(v) or 0 for k, v in fans.items()}
            low_val = p["min"] if p["min"] is not None else 0
            try:
                write_sysfs(pwm_path, low_val)
            except OSError:
                low_val = 255  # driver refused; delta will be ~0
            time.sleep(hold)
            low = {k: read_sysfs_int(v) or 0 for k, v in fans.items()}
        finally:
            try:
                write_sysfs(pwm_path, orig_pwm)
                write_sysfs(en_path, orig_en)
            except OSError as err:
                print(f"WARNING: could not restore {e['hwmon']}/pwm{n}: {err}")
        deltas = {k: high[k] - low[k] for k in fans}
        best = max(deltas, key=lambda k: deltas[k]) if deltas else None
        if best is not None and deltas[best] >= 100:
            print(f"{e['hwmon']} ({e['name']}) pwm{n}: controls fan{best} "
                  f"({low[best]} -> {high[best]} rpm)")
            results.append((e, p, best))
        else:
            detail = ", ".join(f"fan{k}: {deltas[k]}" for k in sorted(deltas)) or "no fans"
            print(f"{e['hwmon']} ({e['name']}) pwm{n}: no clear fan reaction ({detail})")

    if results:
        print("\nsuggested config (verify!):")
        for i, (e, p, fan_n) in enumerate(results, 1):
            dev = os.path.dirname(p["path"])
            print(f"""
[fan{i}]
pwm_path    = {p['path']}
enable_path = {p['enable']}
rpm_path    = {os.path.join(dev, f"fan{fan_n}_input")}
min_pwm     = 32
max_pwm     = 255
group       =""")


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="GPU-temperature driven chassis fan controller")
    ap.add_argument("--config", default="/etc/fanctrl.conf")
    ap.add_argument("--once", action="store_true", help="run a single control cycle and exit")
    ap.add_argument("--dry-run", action="store_true", help="log decisions but do not write pwm")
    ap.add_argument("--detect", action="store_true",
                    help="list all pwm/fan channels + suggested config (read-only)")
    ap.add_argument("--probe", metavar="CHIP:PWMN[,..]|all",
                    help="actively test which fan(s) the given pwm channel(s) control")
    ap.add_argument("--probe-hold", type=float, default=3.0,
                    help="seconds to hold each probe state (default 3)")
    args = ap.parse_args()

    logging.basicConfig(
        stream=sys.stderr,
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    if args.detect:
        cmd_detect()
        return
    if args.probe:
        specs = [s.strip() for s in args.probe.split(",") if s.strip()]
        cmd_probe(specs, args.probe_hold)
        return

    cfg = Config(args.config)
    logging.getLogger().setLevel(getattr(logging, cfg.log_level, logging.INFO))
    FanController(cfg, dry_run=args.dry_run).run(once=args.once)


if __name__ == "__main__":
    main()
