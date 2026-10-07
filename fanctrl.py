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
  * ramp limiting (fast ramp-up, slow ramp-down - no exaggerated jumps)
  * fail-safe PWM when no temperature can be read
  * optional fan-RPM monitoring to detect a dead/stuck fan (warning includes temps)
  * periodic status line so the journal always shows what is happening

Usage:
  fanctrl.py [--config /etc/fanctrl.conf] [--once] [--dry-run]

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
        self.on_sensor_error_pwm = int(m.get("on_sensor_error_pwm", "255"))
        self.fan_rpm_min = int(m.get("fan_rpm_min", "0"))      # 0 = disabled
        self.status_every = int(m.get("status_every", "6"))    # 0 = disabled
        self.log_level = m.get("log_level", "info").upper()

        # ---- fans: [fan1], [fan2], ... (legacy: paths in [main]) ----------
        self.fans = []
        fan_sections = {s for s in cp.sections() if re.fullmatch(r"fan\d+", s)}
        if fan_sections:
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
        elif m.get("pwm_path", ""):
            self.fans.append(FanConfig(
                name="fan1",
                pwm_path=m["pwm_path"],
                enable_path=m.get("enable_path", ""),
                rpm_path=m.get("rpm_path", ""),
                min_pwm=int(m.get("min_pwm", "32")),
                max_pwm=int(m.get("max_pwm", "255")),
            ))
        if not self.fans:
            sys.exit("fanctrl: no fan configured (add a [fan1] section with pwm_path)")

        for f in self.fans:
            if not (0 <= f.min_pwm <= f.max_pwm <= 255):
                sys.exit(f"fanctrl: [{f.name}] bad pwm range {f.min_pwm}..{f.max_pwm}")

        # ---- GPUs: [gpu0], [gpu1], ... (legacy: [edge]/[junction]/[mem]) --
        self.gpus = []
        gpu_sections = {s for s in cp.sections() if re.fullmatch(r"gpu\d*", s)}
        if gpu_sections:
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
        else:
            # legacy single-GPU format: [edge]/[junction]/[mem] with 'points'
            curves = {}
            for sensor in SENSORS:
                if not cp.has_section(sensor):
                    continue
                s = cp[sensor]
                if s.get("enabled", "true").strip().lower() in ("0", "false", "no"):
                    continue
                spec = s.get("points", "")
                if spec:
                    curves[sensor] = parse_points(spec, f"[{sensor}]")
            if not curves:
                sys.exit("fanctrl: no temperature sensors configured "
                         "(add [gpu0] with *_points, or legacy [edge]/[junction]/[mem])")
            self.gpus.append(GpuConfig("gpu0", None, curves))

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
    # conventional fallback order for amdgpu without labels
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
        # start at the HIGHEST member pwm (never drop anyone below where they are)
        pws = []
        for m in members:
            p = read_sysfs_int(m.pwm_path, m.min_pwm)
            pws.append(max(0, min(255, p if p is not None else m.min_pwm)))
        self.current_pwm = max(pws) if pws else 0
        self.actual = {m.name: p for m, p in zip(members, pws)}
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

        # resolve GPU hwmon dirs (explicit or auto-assigned in order)
        auto_pool = list(find_amdgpu_hwmons())
        for gpu in cfg.gpus:
            if gpu.hwmon_dir:
                if not os.path.isdir(gpu.hwmon_dir):
                    sys.exit(f"fanctrl: [{gpu.name}] hwmon dir not found: {gpu.hwmon_dir}")
            else:
                if not auto_pool:
                    sys.exit(f"fanctrl: [{gpu.name}] no amdgpu hwmon device available")
                gpu.hwmon_dir = auto_pool.pop(0)

        self.gpus = []
        for gpu in cfg.gpus:
            temp_map = build_temp_map(gpu.hwmon_dir)
            sensors = {n: p for n, p in temp_map.items() if n in gpu.curves}
            missing = [n for n in gpu.curves if n not in temp_map]
            if missing:
                log.warning("[%s] no sysfs sensor(s) for: %s (skipped)",
                            gpu.name, ", ".join(missing))
            if not sensors:
                log.error("[%s] none of the configured sensors are readable - "
                          "GPU disabled", gpu.name)
                continue
            self.gpus.append((gpu, sensors))
            for name, path in sorted(sensors.items()):
                log.info("[%s] %-9s -> %s (hwmon: %s)", gpu.name, name, path, gpu.hwmon_dir)

        if not self.gpus:
            sys.exit("fanctrl: no readable GPU sensor at all")

        # build fan groups: same non-empty group name -> driven together
        by_group = {}
        order = []
        for f in cfg.fans:
            key = f.group if f.group else f.name
            if key not in by_group:
                by_group[key] = []
                order.append(key)
            by_group[key].append(f)
        self.groups = [FanGroup(k, by_group[k]) for k in order]
        for g in self.groups:
            who = "+".join(m.name for m in g.members)
            log.info("[%s] current pwm at startup: %d (members: %s)", g.name, g.current_pwm, who)

        self.consec_errors = 0
        self.status_counter = 0
        self.last_temps_str = ""
        self._stop = False

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

        # ramp limiting: fast up (safety), slow down (no hunting).
        # First cycle after boot/restart applies an upward target immediately -
        # never start under-cooling a hot GPU.
        for g in self.groups:
            if g.first_cycle and target >= g.current_pwm:
                new = target
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
                 "ramp_up=%d ramp_down=%d dry_run=%s",
                 ",".join(g.name for g in self.groups),
                 ",".join(g.name for g, _ in self.gpus),
                 self.cfg.interval, self.cfg.ramp_up_step, self.cfg.ramp_down_step,
                 self.dry_run)
        while not self._stop:
            started = time.monotonic()
            try:
                self.cycle()
            except Exception as e:  # never die on a transient error
                log.exception("cycle failed: %s", e)
            if once:
                break
            # sleep in small slices so SIGTERM is handled promptly
            deadline = started + self.cfg.interval
            while not self._stop and time.monotonic() < deadline:
                time.sleep(0.2)
        log.info("fanctrl stopped (last pwm: %s)",
                 ", ".join(f"{g.name}={g.current_pwm}" for g in self.groups))

    def _handle_stop(self, signum, frame):
        log.info("received signal %d", signum)
        self._stop = True


def main():
    ap = argparse.ArgumentParser(description="GPU-temperature driven chassis fan controller")
    ap.add_argument("--config", default="/etc/fanctrl.conf")
    ap.add_argument("--once", action="store_true", help="run a single control cycle and exit")
    ap.add_argument("--dry-run", action="store_true", help="log decisions but do not write pwm")
    args = ap.parse_args()

    logging.basicConfig(
        stream=sys.stderr,
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    cfg = Config(args.config)
    logging.getLogger().setLevel(getattr(logging, cfg.log_level, logging.INFO))
    FanController(cfg, dry_run=args.dry_run).run(once=args.once)


if __name__ == "__main__":
    main()
