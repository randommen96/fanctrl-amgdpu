#!/usr/bin/env python3
"""Local tests for fanctrl.py: curve math, legacy + new config formats,
multi-GPU, synced fan groups, rpm warning w/ temps, status lines, fail-safe."""
import importlib.util, logging, os, shutil, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("fanctrl", os.path.join(HERE, "fanctrl.py"))
fc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fc)

# capture log records for assertions
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

# --- 1. curve math ----------------------------------------------------------
pts = fc.parse_points("45:32, 60:192, 70:255", "t")
assert [fc.curve_value(pts, t) for t in (30, 45, 55, 60, 65, 70, 85)] == \
       [32, 32, 139, 192, 224, 255, 255]
print("1. curve math OK")

# --- 2. legacy config format (paths in [main], [edge]/[junction]/[mem]) -----
tmp = tempfile.mkdtemp()
gpuA = make_gpu(tmp, "hwmonA")
fanA = make_fan(tmp, "fanA", pwm="255")
conf = os.path.join(tmp, "legacy.conf")
open(conf, "w").write(f"""
[main]
pwm_path = {fanA}/pwm
enable_path = {fanA}/en
rpm_path = {fanA}/rpm
interval = 1
min_pwm = 32
max_pwm = 255
ramp_up_step = 64
ramp_down_step = 8
on_sensor_error_pwm = 255
fan_rpm_min = 500
status_every = 3

[edge]
enabled = true
points = 45:32, 65:192, 80:255
[junction]
enabled = true
points = 45:32, 60:192, 70:255
[mem]
enabled = true
points = 45:32, 70:192, 90:255
""")
fc.find_amdgpu_hwmons = lambda: [gpuA]
cfg = fc.Config(conf)
assert len(cfg.fans) == 1 and len(cfg.gpus) == 1
ctl = fc.FanController(cfg)
records.clear()
for _ in range(40):
    ctl.cycle()
assert read_fan_pwm(tmp, "fanA") == 32, read_fan_pwm(tmp, "fanA")
statuses = [r for r in records if " status: " in r[1]]
assert len(statuses) >= 10, f"only {len(statuses)} status lines"
assert all("gpu0[" in s[1] and "fan1:pwm=" in s[1] for s in statuses), statuses[:2]
print("2. legacy config OK; idle -> pwm 32; status lines:", len(statuses))

# --- 3. new format: 2 GPUs + 2 fans in sync group 'duct' --------------------
tmp2 = tempfile.mkdtemp()
gpuB = make_gpu(tmp2, "hwmonB")
gpuC = make_gpu(tmp2, "hwmonC")
fanB = make_fan(tmp2, "fanB", pwm="255")   # starts high
fanC = make_fan(tmp2, "fanC", pwm="32")    # starts low -> gets synced up
conf2 = os.path.join(tmp2, "new.conf")
open(conf2, "w").write(f"""
[main]
interval = 1
ramp_up_step = 64
ramp_down_step = 8
on_sensor_error_pwm = 255
fan_rpm_min = 500
status_every = 3

[fan1]
pwm_path = {fanB}/pwm
enable_path = {fanB}/en
rpm_path = {fanB}/rpm
min_pwm = 32
max_pwm = 255
group = duct

[fan2]
pwm_path = {fanC}/pwm
enable_path = {fanC}/en
rpm_path = {fanC}/rpm
min_pwm = 32
max_pwm = 255
group = duct

[gpu0]
edge_points = 45:32, 65:192, 80:255
junction_points = 45:32, 60:192, 70:255
mem_points = 45:32, 70:192, 90:255

[gpu1]
hwmon = {gpuC}
junction_points = 45:32, 60:192, 70:255
""")
fc.find_amdgpu_hwmons = lambda: [gpuB, gpuC]
cfg2 = fc.Config(conf2)
assert len(cfg2.fans) == 2 and len(cfg2.gpus) == 2
ctl2 = fc.FanController(cfg2)
g = ctl2.groups[0]
assert g.name == "duct" and len(g.members) == 2
assert g.current_pwm == 255, "group must start at highest member pwm"

records.clear()
# idle: ramp down together; both fans always identical
for i in range(30):
    ctl2.cycle()
    assert read_fan_pwm(tmp2, "fanB") == read_fan_pwm(tmp2, "fanC"), \
        f"cycle {i}: fans out of sync"
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
assert all(b - a <= 64 for a, b in zip(seq, seq[1:]) if b > a), "ramp step exceeded"
dom = [r for r in records if "pwm" in r[1] and "->" in r[1]]
assert any("gpu1/junction" in r[1] for r in dom), dom[:3]

# --- 4. rpm warning includes temps -------------------------------------------
open(os.path.join(fanC, "rpm"), "w").write("0\n")
records.clear()
for _ in range(5):
    ctl2.cycle()
warns = [r for r in records if r[0] == "WARNING" and "fan rpm" in r[1]]
assert warns, "no rpm warning logged"
assert "gpu1[junction=71.0]" in warns[0][1], warns[0][1]
print("4. rpm warning includes temps OK:", warns[0][1][:100], "...")

# --- 5. fail-safe when all sensors die ---------------------------------------
for d in (gpuB, gpuC):
    for f in ("temp1_input", "temp2_input", "temp3_input"):
        p = os.path.join(d, f)
        if os.path.exists(p):
            os.remove(p)
g.current_pwm = 32
ctl2.cycle()
assert g.current_pwm == 255 and read_fan_pwm(tmp2, "fanB") == 255
print("5. fail-safe on total sensor loss OK")

# --- 6. first-cycle immediate apply (upward) ---------------------------------
for d, j in ((gpuB, "30000"), (gpuC, "71000")):
    open(os.path.join(d, "temp2_input"), "w").write(j + "\n")
ctl3 = fc.FanController(cfg2)
assert len(ctl3.groups) == 1
ctl3.groups[0].current_pwm = 32
records.clear()
ctl3.cycle()
assert read_fan_pwm(tmp2, "fanB") == 255, "first-cycle upward apply failed"
print("6. first-cycle immediate apply OK")

shutil.rmtree(tmp); shutil.rmtree(tmp2)
print("\nALL LOCAL TESTS PASSED")
