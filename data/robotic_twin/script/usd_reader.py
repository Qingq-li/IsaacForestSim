from pxr import Usd

usd_path = "/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd"

stage = Usd.Stage.Open(usd_path)
if not stage:
    raise RuntimeError(f"Failed to open USD: {usd_path}")

default_prim = stage.GetDefaultPrim()
print("Default Prim:", default_prim.GetPath() if default_prim else None)
print("Start Time:", stage.GetStartTimeCode())
print("End Time:", stage.GetEndTimeCode())
print("Root Prims:", [str(p.GetPath())
      for p in stage.GetPseudoRoot().GetChildren()])

print("\nPrim Tree:")
for prim in stage.Traverse():
    print(f"{prim.GetPath()} | type={prim.GetTypeName()} | active={prim.IsActive()}")
