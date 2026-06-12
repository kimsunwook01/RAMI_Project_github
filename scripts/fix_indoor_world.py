import os
import xml.etree.ElementTree as ET

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    path = os.path.join(PROJECT_ROOT, "config/indoor_description/indoor_world.xml")
    tree = ET.parse(path)
    root = tree.getroot()

    for mesh in root.findall(".//mesh"):
        if mesh.get("name") == "base_link":
            mesh.set("name", "room_base_link")

    for body in root.findall(".//body"):
        if body.get("name") == "base_link":
            body.set("name", "room_base_link")

    for geom in root.findall(".//geom"):
        if geom.get("mesh") == "base_link":
            geom.set("mesh", "room_base_link")

    tree.write(path, encoding="utf-8", xml_declaration=True)
    print("Fixed indoor_world.xml base_link clash!")

if __name__ == "__main__":
    main()
