import os
import mujoco
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


def main():
    model = mujoco.MjModel.from_xml_path(os.path.join(PROJECT_ROOT, "config/rami_description/rami_phase4_world.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    
    pnt = np.array([0.0, 0.0, 0.5])
    vec = np.array([1.0, 0.0, 0.0])
    
    try:
        # Let's see what mj_ray returns
        dist = mujoco.mj_ray(model, data, pnt, vec, b"", 1, -1, None)
        print("Dist:", dist)
    except Exception as e:
        print("Error:", e)

if __name__ == "__main__":
    main()
