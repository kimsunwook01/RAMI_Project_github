import os
import re
import xml.etree.ElementTree as ET
import mujoco

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    root_dir = PROJECT_ROOT
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
                if not os.path.exists(filepath) or link.get("name", "").lower().startswith("lamp"):
                    if link.get("name", "").lower().startswith("lamp"):
                        link_name = link.get("name")
                        # 1. Parse size from name (e.g., lamp_100*300_...)
                        parts = link_name.split("_")
                        geom_type = "box"
                        geom_attrs = {"size": "0.1 0.1 0.02"} # default
                        
                        for p in parts:
                            if "*" in p:
                                dims = p.split("*")
                                if len(dims) == 2:
                                    try:
                                        w = float(dims[0]) / 1000.0
                                        l = float(dims[1]) / 1000.0
                                        geom_attrs = {"size": f"{w} {l} 0.02"}
                                    except:
                                        pass
                                break
                            elif p.startswith("r") and p[1:].isdigit():
                                try:
                                    r = float(p[1:]) / 1000.0
                                    geom_type = "cylinder"
                                    geom_attrs = {"radius": str(r), "length": "0.02"}
                                except:
                                    pass
                                break
                        
                        # 2. Replace visual/collision mesh with primitive, and fix origins
                        inertial = link.find("inertial")
                        i_origin = inertial.find("origin") if inertial is not None else None
                        
                        for vis_or_col in link.findall("visual") + link.findall("collision"):
                            geom = vis_or_col.find("geometry")
                            if geom is not None:
                                for child in list(geom):
                                    geom.remove(child)
                                ET.SubElement(geom, geom_type, geom_attrs)
                            
                            origin = vis_or_col.find("origin")
                            if origin is not None and i_origin is not None:
                                origin.set("xyz", i_origin.get("xyz", "0 0 0"))
                                origin.set("rpy", i_origin.get("rpy", "0 0 0"))
                                
                        print(f"Notice: Replaced lamp mesh '{link_name}' with primitive {geom_type} attrs {geom_attrs}")
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

    door_bodies = []
    handle_joints = {}
    latch_joints = {}
    latch_bodies = []
    handle_bodies = []

    # Post-process Joints (add damping, frictionloss, springref, limits)
    for body in root.findall(".//body"):
        bname = body.get("name", "")
        for joint in body.findall("joint"):
            jname = joint.get("name", "")
            if "range" in joint.attrib:
                joint.set("limited", "true")
            
            if "door_" in jname and "_joint" in jname and "handle" not in jname:
                # Main doors (여닫이 문)
                joint.set("frictionloss", "0.5")
                joint.set("damping", "2.0")
                door_bodies.append(bname)
            elif "door_handle" in jname:
                # Door handles (스프링 복원력 추가)
                joint.set("frictionloss", "0.1")
                joint.set("damping", "0.5")
                joint.set("stiffness", "50.0")
                joint.set("springref", "0")
                # extract base door name (e.g. door_1_1)
                base_door = bname.replace("door_handle_", "door_")
                handle_joints[base_door] = jname
                handle_bodies.append(bname)
            elif "latch" in jname:
                # Latches (스프링 탄성으로 항상 튀어나오게 설정)
                joint.set("frictionloss", "0.5")
                joint.set("damping", "1.0")
                joint.set("stiffness", "50.0")
                joint.set("springref", "0")
                base_door = bname.replace("latch_", "door_")
                latch_joints[base_door] = jname
                latch_bodies.append(bname)
            elif "toggle_switch" in jname:
                # Toggle switches
                joint.set("frictionloss", "0.2")
                joint.set("damping", "0.1")
                # 스위치가 벽을 뚫고 거대한 원을 그리며 회전하는 문제(원거리 Pivot) 해결
                # 관성 중심(COM)을 조인트 회전축의 중심(pos)으로 설정하여 제자리에서 회전하도록 함
                parent_body = None
                for body2 in root.findall(".//body"):
                    for j in body2.findall("joint"):
                        if j.get("name") == jname:
                            parent_body = body2
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
                for body2 in root.findall(".//body"):
                    for j in body2.findall("joint"):
                        if j.get("name") == jname:
                            parent_body = body2
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
    
    # Add a QR code texture and material
    ET.SubElement(asset, "texture", {"name": "tex_qr", "type": "2d", "builtin": "checker", 
                                     "rgb1": "0 0 0", "rgb2": "1 1 1", 
                                     "width": "128", "height": "128", "mark": "none"})
    ET.SubElement(asset, "material", {"name": "mat_qr", "texture": "tex_qr", "texrepeat": "5 5", "emission": "0.5"})

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
        elif "lamp" in ident or geom_type in ["box", "cylinder"] and not ident:
            # If we don't have an ident but it's a primitive, it must be the lamp!
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

    # --- Add Lights for Lamps & QR Codes for Switches ---
    worldbody = root.find("worldbody")
    lamp_id = 1
    
    # 1. Add lights to lamp positions
    for geom in worldbody.findall(".//geom"):
        if geom.get("material") == "mat_lamp":
            pos = geom.get("pos", "0 0 0")
            px, py, pz = map(float, pos.split())
            # Place light slightly below the lamp so it casts downward
            light_pos = f"{px} {py} {pz - 0.02}"
            
            ET.SubElement(worldbody, "light", {
                "name": f"light_lamp_{lamp_id}",
                "pos": light_pos,
                "dir": "0 0 -1",
                "directional": "false",
                "castshadow": "true",
                "diffuse": "1.2 1.2 0.8",
                "specular": "0.5 0.5 0.5",
                "attenuation": "1 0 0",
                "cutoff": "90",
                "exponent": "2"
            })
            lamp_id += 1

    # 2. Add QR markers for switches & Build JSON mapping
    import qrcode
    from collections import defaultdict
    import math

    switch_groups = []
    toggles_list = []
    switch_mappings = []
    for body in worldbody.findall(".//body"):
        bname = body.get("name", "")
        if "switch" in bname and "case" not in bname:
            b_pos = body.get("pos", "0 0 0")
            bx, by, bz = map(float, b_pos.split())
            
            inertial = body.find("inertial")
            if inertial is not None:
                i_pos = inertial.get("pos", "0 0 0")
                ix, iy, iz = map(float, i_pos.split())
                global_pos = (bx + ix, by + iy, bz + iz)
            else:
                global_pos = (bx, by, bz)
                
            # Simple mapping logic: extract room name
            room = "unknown"
            if "living-room" in bname: room = "living-room"
            elif "corridor" in bname: room = "corridor"
            elif "kitchen" in bname: room = "kitchen"
            elif "room" in bname: room = "room"
            
            # Find an existing group
            found_group = None
            for g in switch_groups:
                gx, gy, gz = g["avg_pos"]
                if math.hypot(global_pos[0] - gx, global_pos[1] - gy) < 0.2:
                    found_group = g
                    break
                    
            if found_group:
                found_group["toggles"].append(bname)
                found_group["rooms"].append(room)
                # Update max Z
                found_group["max_z"] = max(found_group["max_z"], global_pos[2])
                # Update avg X, Y
                n = len(found_group["toggles"])
                found_group["avg_pos"] = (
                    (found_group["avg_pos"][0] * (n-1) + global_pos[0]) / n,
                    (found_group["avg_pos"][1] * (n-1) + global_pos[1]) / n,
                    found_group["max_z"]
                )
            else:
                new_group = {
                    "id": f"switch_case_{len(switch_groups)+1}",
                    "toggles": [bname],
                    "rooms": [room],
                    "avg_pos": global_pos,
                    "max_z": global_pos[2]
                }
                switch_groups.append(new_group)
                found_group = new_group
            
            toggles_list.append({"name": bname, "case_id": found_group["id"]})

    qr_dir = os.path.join(root_dir, "indoor_space_urdf_description/meshes/qr_codes")
    os.makedirs(qr_dir, exist_ok=True)
    
    for i, group in enumerate(switch_groups):
        case_id = group["id"]
        qr_name = f"qr_{case_id}"
        
        # 10cm above the highest toggle
        gx, gy, gz = group["avg_pos"]
        qr_pos = f"{gx} {gy} {group['max_z'] + 0.10}"
        
        # Generate real QR code image
        qr_data = case_id
        qr = qrcode.QRCode(version=1, box_size=10, border=2)
        qr.add_data(qr_data)
        qr.make(fit=True)
        img = qr.make_image(fill_color="black", back_color="white")
        img_path = os.path.join(qr_dir, f"{case_id}.png")
        img.save(img_path)
        
        # Add texture and material to asset
        tex_name = f"tex_{qr_name}"
        mat_name = f"mat_{qr_name}"
        ET.SubElement(asset, "texture", {"name": tex_name, "type": "cube", "file": img_path.replace("\\", "/")})
        ET.SubElement(asset, "material", {"name": mat_name, "texture": tex_name, "emission": "0.5"})
        
        # Add geom
        ET.SubElement(worldbody, "geom", {
            "name": qr_name,
            "type": "box",
            "size": "0.02 0.02 0.02",
            "pos": qr_pos,
            "material": mat_name
        })
        
        # Map all toggles in this case
        for toggle, room in zip(group["toggles"], group["rooms"]):
            switch_mappings.append({
                "switch_id": toggle,
                "target_room": room,
                "qr_code_id": qr_name,
                "switch_case_id": case_id
            })
            
    # Write JSON metadata
    import json
    config_dir = os.path.join(root_dir, "config")
    os.makedirs(config_dir, exist_ok=True)
    json_path = os.path.join(config_dir, "user_config.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump({"switches": switch_mappings}, f, indent=4, ensure_ascii=False)
    print(f"Created metadata mapping at: {json_path}")

    # Add <contact> section to disable collisions
    contact = ET.SubElement(root, "contact")
    for t in toggles_list:
        ET.SubElement(contact, "exclude", {"body1": t["name"], "body2": "world"})
        
    for lb in latch_bodies:
        # Exclude latch from its handle to prevent internal jamming
        # Since latch and handle belong to the same door, they are siblings or cousins
        for hb in handle_bodies:
            if lb.replace("latch", "") == hb.replace("door_handle", ""):
                ET.SubElement(contact, "exclude", {"body1": lb, "body2": hb})
                
    # Add <equality> section to couple door handles and latches
    equality = ET.SubElement(root, "equality")
    for base_door in handle_joints:
        if base_door in latch_joints:
            h_j = handle_joints[base_door]
            l_j = latch_joints[base_door]
            # Handle rotates 0.785 rad, Latch translates 0.04m -> polycoef = [0, -0.04/0.785, 0, 0, 0] = [0, -0.0509, 0, 0, 0]
            ET.SubElement(equality, "joint", {
                "joint1": l_j, 
                "joint2": h_j, 
                "polycoef": "0 -0.051 0 0 0"
            })
            
    # Add <actuator> section for toggle switch bistable snaps
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
        
    for t in toggles_list:
        # Add negative stiffness via affine bias.
        # biasprm="0 K 0", where K > 0 creates a force in the direction of displacement.
        # gainprm="0" disables the control input from having any effect.
        # We need the joint name. Toggle switches usually have "_joint" appended to body name.
        t_joint = t["name"] + "_joint"
        ET.SubElement(actuator, "general", {
            "joint": t_joint,
            "biastype": "affine",
            "gainprm": "0",
            "biasprm": "0 10.0 0"
        })

    tree.write(out_mjcf_path, encoding="utf-8", xml_declaration=True)

    print("Conversion complete! Output saved to:", out_mjcf_path)

    # Cleanup temp file
    if os.path.exists(temp_urdf_path):
        os.remove(temp_urdf_path)

if __name__ == "__main__":
    main()
