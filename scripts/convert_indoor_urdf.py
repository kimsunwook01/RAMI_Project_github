import os
import re
import xml.etree.ElementTree as ET
import mujoco

def main():
    root_dir = "d:/Programming/RAMI_Project"
    xacro_path = os.path.join(root_dir, "indoor_space_urdf_description/urdf/indoor_space_urdf.xacro")
    temp_urdf_path = os.path.join(root_dir, "temp_indoor.urdf")
    out_dir = os.path.join(root_dir, "config/indoor_description")
    out_mjcf_path = os.path.join(out_dir, "indoor_world.xml")

    # 1. Ensure output directory exists
    os.makedirs(out_dir, exist_ok=True)

    # 2. Read xacro and clean it up
    with open(xacro_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Remove xacro includes
    content = re.sub(r'<xacro:include.*?>', '', content)

    # Replace file://$(find ...) with absolute path
    abs_mesh_dir = os.path.join(root_dir, "indoor_space_urdf_description").replace("\\", "/")
    content = content.replace("file://$(find indoor_space_urdf_description)", abs_mesh_dir)

    # Remove xmlns:xacro just in case
    content = content.replace('xmlns:xacro="http://www.ros.org/wiki/xacro"', '')

    # Parse XML and remove links with missing meshes
    try:
        urdf_root = ET.fromstring(content)
        missing_links = set()
        for link in urdf_root.findall("link"):
            for mesh in link.findall(".//mesh"):
                filename = mesh.get("filename", "")
                if filename.startswith("file://"):
                    filepath = filename[7:]
                else:
                    filepath = filename
                if not os.path.exists(filepath):
                    if "lamp" in filename.lower():
                        link_name = link.get("name")
                        if "*" in link_name:
                            # 1. Parse size from name (e.g., lamp_100*300_...)
                            parts = link_name.split("_")
                            size_str = "0.1 0.1 0.02" # default
                            for p in parts:
                                if "*" in p:
                                    dims = p.split("*")
                                    if len(dims) == 2:
                                        try:
                                            w = float(dims[0]) / 1000.0
                                            l = float(dims[1]) / 1000.0
                                            size_str = f"{w} {l} 0.02"
                                        except:
                                            pass
                                    break
                            
                            # 2. Replace visual/collision mesh with box, and fix origins
                            inertial = link.find("inertial")
                            i_origin = inertial.find("origin") if inertial is not None else None
                            
                            for vis_or_col in link.findall("visual") + link.findall("collision"):
                                geom = vis_or_col.find("geometry")
                                if geom is not None:
                                    for child in list(geom):
                                        geom.remove(child)
                                    ET.SubElement(geom, "box", {"size": size_str})
                                
                                origin = vis_or_col.find("origin")
                                if origin is not None and i_origin is not None:
                                    origin.set("xyz", i_origin.get("xyz", "0 0 0"))
                                    origin.set("rpy", i_origin.get("rpy", "0 0 0"))
                                    
                            print(f"Warning: Replaced missing mesh '{link_name}' with primitive box of size {size_str}")
                            break # Move to next link
                        else:
                            # 대체용 전등 파일 경로로 덮어쓰기
                            fallback_path = os.path.join(root_dir, "indoor_space_urdf_description/meshes/lamp_r50_kitchen_1_1.stl").replace("\\", "/")
                            mesh.set("filename", fallback_path)
                            print(f"Warning: Mesh not found, but substituted with fallback '{link_name}': {filepath}")
                    else:
                        print(f"Warning: Mesh not found, removing link '{link.get('name')}': {filepath}")
                        missing_links.add(link.get("name"))
                        urdf_root.remove(link)
                        break
        
        # Remove joints that reference missing links
        for joint in urdf_root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            if (parent is not None and parent.get("link") in missing_links) or \
               (child is not None and child.get("link") in missing_links):
                print(f"Removing joint '{joint.get('name')}' because it references a missing link.")
                urdf_root.remove(joint)
                
        # Convert back to string
        content = ET.tostring(urdf_root, encoding="unicode")
    except Exception as e:
        print("XML Parsing error during cleanup:", e)

    # Save temp URDF
    with open(temp_urdf_path, "w", encoding="utf-8") as f:
        f.write(content)

    # 3. Load into MuJoCo
    print("Loading temporary indoor URDF into MuJoCo...")
    try:
        model = mujoco.MjModel.from_xml_path(temp_urdf_path)
    except Exception as e:
        print("Error loading URDF:", e)
        return

    # 4. Save to MJCF
    print("Saving to MJCF...")
    mujoco.mj_saveLastXML(out_mjcf_path, model)

    # 5. Post-process MJCF to optimize groups and joints
    print("Optimizing groups and joint properties...")
    tree = ET.parse(out_mjcf_path)
    root = tree.getroot()

    # Add collision group 3 (invisible by default to human observer, but collision enabled)
    # We will assign this to the ceiling.
    # Find all geom tags inside the worldbody
    for geom in root.findall(".//geom"):
        mesh_name = geom.get("mesh", "")
        if "ceiling" in mesh_name.lower():
            # Set the visual geom of ceiling to group 3
            if geom.get("class", "") != "collision":
                geom.set("group", "3")

    # Post-process Joints (add damping, frictionloss, springref, limits)
    for joint in root.findall(".//joint"):
        jname = joint.get("name", "")
        if "door_" in jname and "_joint" in jname and "handle" not in jname:
            # Main doors (여닫이 문)
            joint.set("frictionloss", "0.5")
            joint.set("damping", "2.0")
        elif "door_handle" in jname:
            # Door handles (스프링 복원력 추가)
            joint.set("frictionloss", "0.1")
            joint.set("damping", "0.5")
            joint.set("stiffness", "50.0")
            joint.set("springref", "0")
        elif "latch" in jname:
            # Latches (스프링 탄성으로 항상 튀어나오게 설정)
            joint.set("frictionloss", "0.5")
            joint.set("damping", "1.0")
            joint.set("stiffness", "50.0")
            joint.set("springref", "0")
        elif "toggle_switch" in jname:
            # Toggle switches
            joint.set("frictionloss", "0.2")
            joint.set("damping", "0.1")
            # 스위치가 벽을 뚫고 거대한 원을 그리며 회전하는 문제(원거리 Pivot) 해결
            # 관성 중심(COM)을 조인트 회전축의 중심(pos)으로 설정하여 제자리에서 회전하도록 함
            parent_body = None
            for body in root.findall(".//body"):
                for j in body.findall("joint"):
                    if j.get("name") == jname:
                        parent_body = body
                        break
                if parent_body is not None:
                    break
            
            if parent_body is not None:
                inertial = parent_body.find("inertial")
                if inertial is not None:
                    joint.set("pos", inertial.get("pos", "0 0 0"))
        elif "key_switch" in jname:
            # Key switches
            joint.set("frictionloss", "0.2")
            joint.set("damping", "0.5")
            joint.set("stiffness", "10.0")
            joint.set("springref", "0")
            
            parent_body = None
            for body in root.findall(".//body"):
                for j in body.findall("joint"):
                    if j.get("name") == jname:
                        parent_body = body
                        break
                if parent_body is not None:
                    break
                    
            if parent_body is not None:
                inertial = parent_body.find("inertial")
                if inertial is not None:
                    joint.set("pos", inertial.get("pos", "0 0 0"))

    # 6. Apply Materials and Colors
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    
    # Add a wooden floor texture (checkerboard of two dark wood colors)
    ET.SubElement(asset, "texture", {"name": "tex_wood", "type": "2d", "builtin": "checker", 
                                     "rgb1": "0.3 0.15 0.05", "rgb2": "0.2 0.1 0.02", 
                                     "width": "512", "height": "512", 
                                     "mark": "edge", "markrgb": "0.1 0.05 0.01"})
    ET.SubElement(asset, "material", {"name": "mat_wood", "texture": "tex_wood", 
                                      "texrepeat": "20 20", "reflectance": "0.1"})
                                      
    # Add a glowing material for lamps
    ET.SubElement(asset, "material", {"name": "mat_lamp", "rgba": "0.95 0.95 0.5 1", "emission": "1.0"})

    # Iterate over all geoms. For flattened bodies, the name is lost but mesh attribute remains.
    for geom in root.findall(".//geom"):
        mesh_name = geom.get("mesh", "").lower()
        geom_type = geom.get("type", "")
        
        # Determine the identifier string (either the mesh name, or the parent body name)
        # ElementTree doesn't easily give parent, but we can search for bodies and map them.
        # Actually, if we just look at mesh_name, it covers 90% of objects.
        # The only objects without mesh_name are the lamp boxes.
        ident = mesh_name
        if not ident and geom_type == "box":
            ident = "lamp" # Because we only added boxes for missing lamps!
            
        # For geoms that are inside a body, we can also check the body name.
        # But we don't have parent links. Let's just do a second pass for bodies.
        
        rgba = None
        emission = None
        material = None
        
        if "wall" in ident:
            rgba = "0.85 0.85 0.85 1" # 밝은 회색
        elif "ceiling" in ident:
            rgba = "0.85 0.85 0.85 1" # 밝은 회색
        elif "base_link" in ident or "floor" in ident:
            material = "mat_wood"
        elif "door" in ident and "handle" not in ident and "latch" not in ident:
            rgba = "0.7 0.5 0.3 1" # 밝은 갈색
        elif "handle" in ident or "latch" in ident or "switch" in ident:
            rgba = "0.6 0.6 0.6 1" # 회색
        elif "lamp" in ident:
            material = "mat_lamp"
            
        if rgba or material:
            if rgba and not material:
                if "material" in geom.attrib:
                    del geom.attrib["material"]
                geom.set("rgba", rgba)
                # Ensure no emission on geom
                if "emission" in geom.attrib:
                    del geom.attrib["emission"]
            elif material:
                geom.set("material", material)
                if "rgba" in geom.attrib:
                    del geom.attrib["rgba"]
                if "emission" in geom.attrib:
                    del geom.attrib["emission"]

    # Second pass for geoms inside named bodies (to catch doors/switches if mesh name isn't enough)
    for body in root.findall(".//body"):
        bname = body.get("name", "").lower()
        rgba = None
        emission = None
        material = None
        
        if "wall" in bname:
            rgba = "0.85 0.85 0.85 1"
        elif "ceiling" in bname:
            rgba = "0.85 0.85 0.85 1"
        elif "base_link" in bname or "floor" in bname:
            material = "mat_wood"
        elif "door" in bname and "handle" not in bname and "latch" not in bname:
            rgba = "0.7 0.5 0.3 1"
        elif "handle" in bname or "latch" in bname or "switch" in bname:
            rgba = "0.6 0.6 0.6 1"
        elif "lamp" in bname:
            material = "mat_lamp"
            
        for geom in body.findall("geom"):
            if rgba and not material:
                if "material" in geom.attrib:
                    del geom.attrib["material"]
                geom.set("rgba", rgba)
                if "emission" in geom.attrib:
                    del geom.attrib["emission"]
            elif material:
                geom.set("material", material)
                if "rgba" in geom.attrib:
                    del geom.attrib["rgba"]
                if "emission" in geom.attrib:
                    del geom.attrib["emission"]

    tree.write(out_mjcf_path, encoding="utf-8", xml_declaration=True)

    print("Conversion complete! Output saved to:", out_mjcf_path)

    # Cleanup temp file
    if os.path.exists(temp_urdf_path):
        os.remove(temp_urdf_path)

if __name__ == "__main__":
    main()
