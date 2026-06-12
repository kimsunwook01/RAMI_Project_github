import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mujoco
from src.infrastructure.simulator.rami_mujoco_adapter import RamiMujocoAdapter

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_arm():
    model = mujoco.MjModel.from_xml_path(os.path.join(PROJECT_ROOT, "config/rami_description/rami_world.xml"))
    data = mujoco.MjData(model)
    adapter = RamiMujocoAdapter(model, data)
    
    # 1. Init Base Position
    root_x_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_x")
    root_y_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root_y")
    if root_x_id != -1 and root_y_id != -1:
        data.qpos[model.jnt_qposadr[root_x_id]] = 10.0
        data.qpos[model.jnt_qposadr[root_y_id]] = 15.0
    
    # 2. Init Arm Joints
    safe_arm_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    arm_joints = [
        "lift_slide_joint", "rotation_joint",
        "arm_joint_1", "arm_joint_2", "arm_joint_3", 
        "arm_joint_4", "arm_joint_5", "arm_joint_6"
    ]
    
    print("=== Initializing ===")
    for j_name, angle in zip(arm_joints, safe_arm_pose):
        j_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j_name)
        if j_id != -1:
            q_adr = model.jnt_qposadr[j_id]
            data.qpos[q_adr] = angle
            print(f"Set {j_name} (ID: {j_id}, q_adr: {q_adr}) to {angle}")
        else:
            print(f"JOINT NOT FOUND: {j_name}")
            
    # 3. Set Control
    adapter.control_arm_joints(safe_arm_pose)
    
    act_lift_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_lift")
    jnt_id = model.actuator_trnid[act_lift_id, 0]
    jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id)
    print(f"Ctrl for act_lift (ID {act_lift_id}) acts on joint: {jnt_name} (ID {jnt_id}). Value: {data.ctrl[act_lift_id]}")
    
    # 4. Step Simulation
    mujoco.mj_forward(model, data)
    print("\n=== After Forward ===")
    actual = adapter.read_arm_joints()
    for name, act in zip(arm_joints, actual):
        print(f"{name}: {act}")
        
    print("\n=== Stepping 100 times (approx 0.2s) ===")
    for _ in range(100):
        adapter.control_arm_joints(safe_arm_pose)
        mujoco.mj_step(model, data)
        
    print("\n=== After Step ===")
    actual = adapter.read_arm_joints()
    for name, act in zip(arm_joints, actual):
        print(f"{name}: {act}")
        
    print(f"\nActuator Force for act_lift: {data.actuator_force[act_lift_id]}")

if __name__ == "__main__":
    test_arm()
