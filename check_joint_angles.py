import os
import mujoco
import time
import math

def main():
    xml_path = os.path.join(os.path.dirname(__file__), "config", "rami_description", "rami_phase4_world.xml")
    xml_path = os.path.abspath(xml_path)
    print(f"Loading model from {xml_path}")
    
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # Identify joints to monitor
    test_joints = []
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name and ("door" in name or "switch" in name or "latch" in name):
            test_joints.append(name)
            
    print(f"Monitoring {len(test_joints)} joints for 5 seconds...")
    
    # Simulate for 5 seconds (500 steps of 0.01s)
    mujoco.mj_resetData(model, data)
    
    for step in range(500):
        mujoco.mj_step(model, data)
        
        # Print every 100 steps (1 second)
        if step % 100 == 0:
            print(f"--- Time: {step * 0.01:.2f}s ---")
            for jname in test_joints:
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                qpos_idx = model.jnt_qposadr[jid]
                qpos_val = data.qpos[qpos_idx]
                
                limit_str = ""
                if model.jnt_limited[jid]:
                    jmin, jmax = model.jnt_range[jid]
                    limit_str = f" [Limits: {math.degrees(jmin):.1f} ~ {math.degrees(jmax):.1f} deg]"
                
                print(f"{jname}: {math.degrees(qpos_val):.2f} deg {limit_str}")

if __name__ == "__main__":
    main()
