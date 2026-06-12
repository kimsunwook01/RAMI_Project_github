import mujoco
import time
import math

import os
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

def main():
    xml_path = os.path.join(PROJECT_ROOT, "config", "rami_description", "rami_phase4_world.xml")
    print(f"Loading model from {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    door_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "door_1_1_joint")
    latch_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "latch_1_1_joint")
    handle_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "door_handle_1_1_joint")

    door_qpos_adr = model.jnt_qposadr[door_joint_id]
    latch_qpos_adr = model.jnt_qposadr[latch_joint_id]
    handle_qpos_adr = model.jnt_qposadr[handle_joint_id]

    # Open the door initially to -1.0 rad
    data.qpos[door_qpos_adr] = -1.0
    mujoco.mj_forward(model, data)

    print("Starting simulation to close the door slowly...")
    print("Door Angle (deg) | Latch Pos (m) | Handle Angle (deg)")
    print("-" * 55)

    # We will forcefully move the door from -1.0 rad to 0.0 rad over 5000 steps
    door_start = -1.0
    door_end = 0.0
    
    for step in range(5000): # 5 seconds at 1ms timestep
        # Force the door position
        progress = step / 5000.0
        target_door = door_start + (door_end - door_start) * progress
        data.qpos[door_qpos_adr] = target_door
        
        # Zero out door velocity to prevent huge momentum from throwing the simulation off
        door_dof_adr = model.jnt_dofadr[door_joint_id]
        data.qvel[door_dof_adr] = 0.0

        mujoco.mj_step(model, data)

        if step % 100 == 0:
            door_deg = math.degrees(data.qpos[door_qpos_adr])
            latch_pos = data.qpos[latch_qpos_adr]
            handle_deg = math.degrees(data.qpos[handle_qpos_adr])
            print(f"{door_deg:15.2f} | {latch_pos:13.4f} | {handle_deg:17.2f}")

    print("-" * 55)
    print("Final State:")
    print(f"Door Angle: {math.degrees(data.qpos[door_qpos_adr]):.2f} deg")
    print(f"Latch Pos: {data.qpos[latch_qpos_adr]:.4f} m")
    print(f"Handle Angle: {math.degrees(data.qpos[handle_qpos_adr]):.2f} deg")

if __name__ == "__main__":
    main()
