from pxr import Usd

usd_path = "/home/prefor/IsaacForestSim/data/robotic_twin/saha-robot/harveri_sensor_pj_simple.usd"
prim_path = "/World/harveri_isaac_full_0_1/harveri_isaac_flat/BASE/harveri_sensors_0_5/sensor_kit_rb/sensor_kit/ZED_X/base_link/ZED_X/CameraLeft"

stage = Usd.Stage.Open(usd_path)
prim = stage.GetPrimAtPath(prim_path)

print("Prim:", prim.GetPath())
print("Type:", prim.GetTypeName())
print("\nAttributes:")
for attr in prim.GetAttributes():
    print(attr.GetName(), "=", attr.Get())
