#!/usr/bin/env python3
"""Local tests for fanctrl.py: curve math, multi-GPU, synced fan groups,
anti-hunting hysteresis, rpm warning w/ temps, status lines, fail-safe,
GPU-absent (driver not loaded) mode + recovery."""
import importlib.util, logging, os, shutil, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("fanctrl", os.path.join(HERE, "fanctrl.py"))
fc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fc)

records = []
class CapHandler(logging.Handler):
    def emit(self, record):
        records.append((record.levelname, self.format(record)))
cap = CapHandler()
cap.setFormatter(logging.Formatter("%(levelname)s %(message)s"))
logging.getLogger("fanctrl").addHandler(cap)
logging.getLogger("fanctrl").setLevel(logging.DEBUG)

def make_gpu(base, name, edge="27000", junction="28000", mem="25000"):
    d = os.path.join(base, name)
    os.makedirs(d, exist_ok=True)
    def w(f, v): open(os.path.join(d, f), "w").write(v + "\n")
    w("name", "amdgpu")
    for label, val in (("temp1", edge), ("temp2", junction), ("temp3", mem)):
        if val is not None:
            w(f"{label}_label", {"temp1": "edge", "temp2": "junction", "temp3": "mem"}[label])
            w(f"{label}_input", val)
    return d

def make_fan(base, name, pwm="255"):
    d = os.path.join(base, name)
    os.makedirs(d, exist_ok=True)
    open(os.path.join(d, "pwm"), "w").write(pwm + "\n")
    open(os.path.join(d, "en"), "w").write("0\n")
    open(os.path.join(d, "rpm"), "w").write("2909\n")
    return d

def read_fan_pwm(base, name):
    return int(open(os.path.join(base, name, "pwm")).read().strip())

CONF_TMPL = """
[main]
interval = 1
ramp_up_step = 64
ramp_down_step = 8
spin_up_margin = 48
on_sensor_error_pwm = 255
fan_rpm_min = 500
status_every = 3

{fans}
[gpu0]
hwmon = {gpu0}
edge_points = 45:32, 65:192, 80:255
junction_points = 45:32, 60:192, 70:255
mem_points = 45:32, 70:192, 90:255
{gpu1}
"""

def fan_section(base, dir_name, idx, group=""):
    return f"""
[fan{idx}]
pwm_path = {base}/{dir_name}/pwm
enable_path = {base}/{dir_name}/en
rpm_path = {base}/{dir_name}/rpm
min_pwm = 32
max_pwm = 255
group = {group}
"""

# --- 1. curve math ----------------------------------------------------------
pts = fc.parse_points("45:32, 60:192, 70:255", "t")
assert [fc.curve_value(pts, t) for t in (30, 45, 55, 60, 65, 70, 85)] == \
       [32, 32, 139, 192, 224, 255, 255]
print("1. curve math OK")

# --- 2. single fan + anti-hunting hysteresis ---------------------------------
tmp = tempfile.mkdtemp()
gpuA = make_gpu(tmp, "hwmonA")
fanA = make_fan(tmp, "fanA", pwm="32")
conf = os.path.join(tmp, "c.conf")
open(conf, "w").write(CONF_TMPL.format(fans=fan_section(tmp, "fanA", 1), gpu0=gpuA, gpu1=""))
fc.find_amdgpu_hwmons = lambda: [gpuA]
ctl = fc.FanController(fc.Config(conf))
g = ctl.groups[0]

# idle at 27/28/25 -> target 32 == base -> hold at baseline
for _ in range(5):
    ctl.cycle()
assert g.current_pwm == 32, g.current_pwm

# small bump: junction 46 C -> target ~43 < base+margin (32+48=80) -> NO restart
open(os.path.join(gpuA, "temp2_input"), "w").write("46000\n")
records.clear()
for _ in range(5):
    ctl.cycle()
assert g.current_pwm == 32, f"hysteresis failed: pwm {g.current_pwm} at 46C"
print("2a. hysteresis: 46 C (target ~43) does NOT restart a stalled fan")

# bigger bump: junction 50 C -> target ~85 >= 80 -> spin up (one ramp step)
open(os.path.join(gpuA, "temp2_input"), "w").write("50000\n")
ctl.cycle()
assert g.current_pwm == 85, g.current_pwm  # min(target 85, 32 + ramp_up_step 64)
print(f"2b. real demand: 50 C (target ~85) spins up to {g.current_pwm}")

# cools below curve start -> ramps down to baseline...
open(os.path.join(gpuA, "temp2_input"), "w").write("44000\n")
for _ in range(20):
    ctl.cycle()
assert g.current_pwm == 32, g.current_pwm
# ...and a small bump again does NOT restart it (the anti-flapping case)
open(os.path.join(gpuA, "temp2_input"), "w").write("46000\n")
for _ in range(5):
    ctl.cycle()
assert g.current_pwm == 32, g.current_pwm
print("2c. cooled to baseline, small bump again: pinned at 32 (no flapping)")

# --- 3. multi-GPU + synced fan group -----------------------------------------
tmp2 = tempfile.mkdtemp()
gpuB = make_gpu(tmp2, "hwmonB")
gpuC = make_gpu(tmp2, "hwmonC")
fanB = make_fan(tmp2, "fanB", pwm="255")
fanC = make_fan(tmp2, "fanC", pwm="32")
conf2 = os.path.join(tmp2, "c.conf")
open(conf2, "w").write(CONF_TMPL.format(
    fans=fan_section(tmp2, "fanB", 1, group="duct") + fan_section(tmp2, "fanC", 2, group="duct"),
    gpu0=gpuB,
    gpu1=f"[gpu1]\nhwmon = {gpuC}\njunction_points = 45:32, 60:192, 70:255\n"))
fc.find_amdgpu_hwmons = lambda: [gpuB, gpuC]
ctl2 = fc.FanController(fc.Config(conf2))
g2 = ctl2.groups[0]
assert g2.name == "duct" and len(g2.members) == 2 and g2.current_pwm == 255

records.clear()
for i in range(30):
    ctl2.cycle()
    assert read_fan_pwm(tmp2, "fanB") == read_fan_pwm(tmp2, "fanC"), f"cycle {i}: out of sync"
assert read_fan_pwm(tmp2, "fanB") == 32
print("3a. synced group ramped down together to 32, always identical")

# heat gpu1 junction to 71 -> target 255, fast ramp up (+64/cycle)
open(os.path.join(gpuC, "temp2_input"), "w").write("71000\n")
records.clear()
seq = []
for i in range(8):
    ctl2.cycle()
    seq.append(read_fan_pwm(tmp2, "fanB"))
    assert read_fan_pwm(tmp2, "fanB") == read_fan_pwm(tmp2, "fanC"), f"cycle {i}: out of sync"
assert seq[-1] == 255, seq
print("3b. ramp-up sequence (both fans):", seq)
dom = [r for r in records if "->" in r[1]]
assert any("gpu1/junction" in r[1] for r in dom), dom[:3]

# --- 4. rpm warning includes temps -------------------------------------------
open(os.path.join(fanC, "rpm"), "w").write("0\n")
records.clear()
for _ in range(5):
    ctl2.cycle()
warns = [r for r in records if r[0] == "WARNING" and "fan rpm" in r[1]]
assert warns and "gpu1[junction=71.0]" in warns[0][1], warns[:1]
print("4. rpm warning includes temps OK")

# --- 5. fail-safe when all sensors die ---------------------------------------
for d in (gpuB, gpuC):
    for f in ("temp1_input", "temp2_input", "temp3_input"):
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)
g2.current_pwm = 32
ctl2.cycle()
assert g2.current_pwm == 255 and read_fan_pwm(tmp2, "fanB") == 255
print("5. fail-safe on total sensor loss OK")

# --- 6. first-cycle immediate apply (upward) ---------------------------------
for d, j in ((gpuB, "30000"), (gpuC, "71000")):
    open(os.path.join(d, "temp2_input"), "w").write(j + "\n")
ctl3 = fc.FanController(fc.Config(conf2))
ctl3.groups[0].current_pwm = 32
ctl3.cycle()
assert read_fan_pwm(tmp2, "fanB") == 255, "first-cycle upward apply failed"
print("6. first-cycle immediate apply OK")

# --- 7. GPU absent (driver not loaded) -> fail-safe mode, no crash -----------
# conf3 has NO explicit hwmon pin: with the driver unloaded the amdgpu hwmon
# dirs are gone entirely, so auto-assignment finds nothing
conf3 = os.path.join(tmp2, "c3.conf")
open(conf3, "w").write(CONF_TMPL.format(
    fans=fan_section(tmp2, "fanB", 1),
    gpu0="",   # no [gpu0] hwmon pin -> auto-assign from find_amdgpu_hwmons()
    gpu1=""))
# template requires a [gpu0]; rebuild without hwmon line
conf3_txt = open(conf3).read().replace(f"hwmon = {gpuB}\n", "")
open(conf3, "w").write(conf3_txt)
fc.find_amdgpu_hwmons = lambda: []
records.clear()
ctl4 = fc.FanController(fc.Config(conf3))   # must NOT sys.exit
assert ctl4.gpus == [], ctl4.gpus
for _ in range(3):
    ctl4.cycle()                            # must NOT raise
errs = [r for r in records if r[0] == "ERROR"]
assert len(errs) == 1, f"expected 1 error (not per-cycle spam), got {len(errs)}: {errs}"
assert "fail-safe pwm 255" in errs[0][1], errs[0][1]
print("7a. GPU absent: fail-safe mode, single error logged, no crash")

# --- 8. GPU returns (modprobe amdgpu) -> resume normal control ---------------
fc.find_amdgpu_hwmons = lambda: [gpuB, gpuC]   # auto-assign takes the first
records.clear()
ctl4.cycle()
assert len(ctl4.gpus) >= 1, ctl4.gpus
infos = [r for r in records if "resuming normal control" in r[1]]
assert infos, records[:5]
print("8. GPU returned: normal control resumed")

# --- 9. unified log format: FAN | TEMPS | CONTROL -------------------------
import re
# force a pwm change on ctl2's duct group (junction hot -> ramp up)
def msg(r):  # strip the 'LEVEL ' prefix the cap handler adds
    return r[1][len(r[0]) + 1:]

records.clear()
ctl2.cycle()  # may or may not change; ensure a change happens next
g2.current_pwm = g2.current_pwm - 64 if g2.current_pwm > 32 else 32
open(os.path.join(gpuC, "temp2_input"), "w").write("71000\n")
ctl2.cycle()
chg = [r for r in records if re.match(r"^\[duct\] pwm \d+ -> \d+ ", msg(r))]
assert chg, records[:5]
# FAN | TEMPS | CONTROL - same section order as the status line
m = re.fullmatch(r"\[duct\] pwm \d+ -> \d+ \| (gpu\d\[[^\]]*\] )+gpu1\[junction=71\.0\] \| target=255 dominant=gpu1/junction", msg(chg[0]))
assert m, msg(chg[0])
# status line: same section order
ctl2.status_counter = 2  # next cycle hits the status_every=3 boundary
records.clear()
ctl2.cycle()
st = [r for r in records if msg(r).startswith("status: ")]
assert st, records[:5]
ms = re.fullmatch(r"status: duct:pwm=\d+ rpm=\d+/\d+ \| gpu0\[.*\] gpu1\[junction=71\.0\] \| target=255 dominant=gpu1/junction", msg(st[0]))
assert ms, msg(st[0])
print("9. unified log format (FAN | TEMPS | CONTROL) OK")

# --- 10. synced group out-of-sync warning ------------------------------------
g2.sync_state = {"streak": 0, "warned": False}   # test 4 already desynced them
open(os.path.join(fanB, "rpm"), "w").write("2900\n")
open(os.path.join(fanC, "rpm"), "w").write("1500\n")   # 1400 apart >= max(150, 15%)
records.clear()
for _ in range(3):
    ctl2.cycle()
sync_warns = [r for r in records if r[0] == "WARNING" and "out of sync" in msg(r)]
assert len(sync_warns) == 1, sync_warns
assert "fan1=2900 fan2=1500" in sync_warns[0][1], sync_warns[0][1]
# no repeat while still out of sync
for _ in range(3):
    ctl2.cycle()
assert sum(1 for r in records if "out of sync" in msg(r)) == 1, "warned twice"
# normal spread again (~8%, below the 150 rpm floor) -> recovery note
open(os.path.join(fanB, "rpm"), "w").write("2900\n")
open(os.path.join(fanC, "rpm"), "w").write("2870\n")
records.clear()
ctl2.cycle()
rec = [r for r in records if "back in sync" in msg(r)]
assert rec, records[:5]
print("10. out-of-sync warning + recovery OK")

shutil.rmtree(tmp); shutil.rmtree(tmp2)
print("\nALL LOCAL TESTS PASSED")
