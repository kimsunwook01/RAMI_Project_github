import os
import mujoco
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def check_floor():
    model = mujoco.MjModel.from_xml_path(os.path.join(PROJECT_ROOT, "config/rami_description/rami_phase4_world.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    
    pnt = np.array([11.0, 16.0, 1.0])
    vec = np.array([0.0, 0.0, -1.0])
    geomid = np.array([-1], dtype=np.int32)
    
    dist = mujoco.mj_ray(model, data, pnt, vec, None, 1, -1, geomid)
    if dist > 0:
        hit_z = 1.0 - dist
        print(f"Floor hit at Z = {hit_z}")
    else:
        print("No floor hit")

if __name__ == "__main__":
    check_floor()
