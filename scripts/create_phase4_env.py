import os
import re

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    root_dir = PROJECT_ROOT
    rami_world_path = os.path.join(root_dir, "config/rami_description/rami_world.xml")
    phase4_world_path = os.path.join(root_dir, "config/rami_description/rami_phase4_world.xml")

    # 1. Generate standalone phase4 world (Robot + Indoor Space)
    standalone_xml = f"""<?xml version='1.0' encoding='utf-8'?>
<mujoco model="rami_phase4_world">
  <compiler angle="radian" />
  <include file="rami_world.xml" />
  <include file="../indoor_description/indoor_world.xml" />
  
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true" castshadow="false" />
  </worldbody>
</mujoco>
"""

    # 2. Save to phase4 world
    with open(phase4_world_path, "w", encoding="utf-8") as f:
        f.write(standalone_xml)

    print(f"Created standalone {phase4_world_path} successfully!")

if __name__ == "__main__":
    main()
