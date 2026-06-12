import xml.etree.ElementTree as ET
tree = ET.parse('config/rami_description/rami_world.xml')
for g in tree.iter('geom'):
    if 'contype' not in g.attrib:
        print(f"NO CONTYPE: name={g.attrib.get('name', '')} mesh={g.attrib.get('mesh', '')} type={g.attrib.get('type', '')}")
