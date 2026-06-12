import xml.etree.ElementTree as ET
import os

def create_indoor_robot_env():
    # 1. 원본 파일 로드
    base_dir = os.path.dirname(os.path.dirname(__file__))
    rami_world_path = os.path.join(base_dir, 'config', 'rami_description', 'rami_world.xml')
    indoor_world_path = os.path.join(base_dir, 'config', 'indoor_description', 'indoor_world.xml')
    output_path = os.path.join(base_dir, 'config', 'rami_description', 'rami_indoor_world.xml')
    
    rami_tree = ET.parse(rami_world_path)
    rami_root = rami_tree.getroot()
    
    indoor_tree = ET.parse(indoor_world_path)
    indoor_root = indoor_tree.getroot()
    
    # 2. rami_world.xml의 컴파일러 경로 설정
    compiler = rami_root.find('compiler')
    if compiler is not None:
        compiler.set('meshdir', '')
        compiler.set('texturedir', '')
        
    # 3. 에셋 병합 (Mesh, Material, Texture)
    rami_asset = rami_root.find('asset')
    if rami_asset is None:
        rami_asset = ET.SubElement(rami_root, 'asset')
        
    indoor_asset = indoor_root.find('asset')
    if indoor_asset is not None:
        for item in list(indoor_asset):
            if item.tag in ['mesh', 'material', 'texture']:
                if 'file' in item.attrib:
                    pass # Paths are already absolute in indoor_world.xml
                rami_asset.append(item)
                
    # 4. 실내 공간 지오메트리 병합
    rami_worldbody = rami_root.find('worldbody')
    indoor_worldbody = indoor_root.find('worldbody')
    
    if indoor_worldbody is not None:
        for body in list(indoor_worldbody):
            # 조명이나 중복되는 바닥(floor) 제외하고 메쉬만 병합
            if body.tag == 'geom':
                if body.get('name') == 'floor':
                    continue
                rami_worldbody.append(body)
            elif body.tag == 'body':
                rami_worldbody.append(body)
                
    # 5. 저장
    rami_tree.write(output_path, encoding='utf-8', xml_declaration=True)
    print(f"Created merged environment: {output_path}")

if __name__ == '__main__':
    create_indoor_robot_env()
