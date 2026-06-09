import mujoco
import sys
import os
import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

def render_spawn():
    model = mujoco.MjModel.from_xml_path('d:/Programming/RAMI_Project/config/rami_description/rami_indoor_world.xml')
    data = mujoco.MjData(model)
    adapter = RamiMujocoAdapter(model, data)
    
    # Spawn at 0,0
    root_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
    root_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_y")
    if root_x_id != -1 and root_y_id != -1:
        data.qpos[model.jnt_qposadr[root_x_id]] = 0.0
        data.qpos[model.jnt_qposadr[root_y_id]] = 0.0
        
    safe_arm_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    adapter.control_arm_joints(safe_arm_pose)
    
    mujoco.mj_forward(model, data)
    
    for _ in range(100):
        adapter.control_arm_joints(safe_arm_pose)
        mujoco.mj_step(model, data)
        
    # Render using the head camera
    width, height = 640, 480
    renderer = mujoco.Renderer(model, height, width)
    renderer.update_scene(data, camera="head_camera")
    img = renderer.render()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    output_path = "C:/Users/sunny/.gemini/antigravity-ide/brain/27554dc4-80c9-47a9-a5e3-d6d1f0d2e6f8/safe_spawn_head_camera.png"
    cv2.imwrite(output_path, img)
    print(f"Rendered to {output_path}")

if __name__ == '__main__':
    render_spawn()
