import os
import re
import xml.etree.ElementTree as ET
import mujoco

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


def main():
    root_dir = PROJECT_ROOT
    xacro_path = os.path.join(root_dir, "robot_RAMI_v1_urdf_description/urdf/robot_RAMI_v1_urdf.xacro")
    temp_urdf_path = os.path.join(root_dir, "temp_clean.urdf")
    out_dir = os.path.join(root_dir, "config/rami_description")
    out_mjcf_path = os.path.join(out_dir, "rami_world.xml")

    # 1. Ensure output directory exists
    os.makedirs(out_dir, exist_ok=True)

    # 2. Read xacro and clean it up
    with open(xacro_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Remove xacro includes
    content = re.sub(r'<xacro:include.*?>', '', content)

    # Replace file://$(find ...) with absolute path
    abs_mesh_dir = os.path.join(root_dir, "robot_RAMI_v1_urdf_description").replace("\\", "/")
    content = content.replace("file://$(find robot_RAMI_v1_urdf_description)", abs_mesh_dir)

    # Remove xmlns:xacro just in case
    content = content.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', '')

    # Inject a dummy world link and floating joint to force base_link to be a free body
    # This preserves the inertia of base_link and automatically adds a freejoint in MJCF
    world_joint_xml = """
<link name="world"/>
<joint name="world_to_base" type="floating">
  <parent link="world"/>
  <child link="base_link"/>
</joint>
"""
    # Insert right after <robot ...>
    content = re.sub(r'(<robot.*?>)', r'\1\n' + world_joint_xml, content)

    with open(temp_urdf_path, "w", encoding="utf-8") as f:
        f.write(content)

    # 3. Load into MuJoCo
    print("Loading temporary URDF into MuJoCo...")
    try:
        model = mujoco.MjModel.from_xml_path(temp_urdf_path)
    except Exception as e:
        print("Error loading URDF:", e)
        return

    # 4. Save to MJCF using MuJoCo's native XML saver
    print("Saving to MJCF...")
    mujoco.mj_saveLastXML(out_mjcf_path, model)

    # 5. Post-process MJCF to add floor and light
    print("Adding floor and light to worldbody...")
    tree = ET.parse(out_mjcf_path)
    root = tree.getroot()

    worldbody = root.find("worldbody")
    if worldbody is not None:
        # Add light if not exists
        if worldbody.find("light") is None:
            light = ET.Element("light", {"pos": "0 0 3", "dir": "0 0 -1", "directional": "true"})
            worldbody.insert(0, light)
        
        # Add floor if not exists
        if worldbody.find("./geom[@name='floor']") is None:
            floor = ET.Element("geom", {
                "name": "floor", 
                "type": "plane", 
                "size": "5 5 0.05", 
                "rgba": "0.8 0.9 0.8 1"
            })
            worldbody.insert(1, floor)
            
    tree.write(out_mjcf_path, encoding="utf-8", xml_declaration=True)

    print("Conversion complete! Output saved to:", out_mjcf_path)

    # Cleanup temp file
    if os.path.exists(temp_urdf_path):
        os.remove(temp_urdf_path)

if __name__ == "__main__":
    main()
