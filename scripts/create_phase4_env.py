import os
import re

def main():
    root_dir = "d:/Programming/RAMI_Project"
    rami_world_path = os.path.join(root_dir, "config/rami_description/rami_world.xml")
    phase4_world_path = os.path.join(root_dir, "config/rami_description/rami_phase4_world.xml")

    with open(rami_world_path, "r", encoding="utf-8") as f:
        content = f.read()

    # 1. Remove Test Objects
    # Remove cylinder
    content = re.sub(r'<!-- Test 18-1.*?-->\s*<body name="test_object_cylinder".*?</body>', '', content, flags=re.DOTALL)
    # Remove box
    content = re.sub(r'<body name="test_object_box".*?</body>', '', content, flags=re.DOTALL)
    # Remove sphere
    content = re.sub(r'<!-- Test 18-2.*?-->\s*<body name="test_object_sphere".*?</body>', '', content, flags=re.DOTALL)

    # 2. Remove default floor and light since indoor_world has its own enclosed space
    # (We might keep the light, but let's remove the floor to avoid z-fighting)
    content = re.sub(r'<!-- 바닥.*?-->\s*<geom name="floor".*?/>', '', content, flags=re.DOTALL)

    # 3. Include indoor_world.xml
    # Insert right after <compiler angle="radian" />
    include_str = '\n  <include file="../indoor_description/indoor_world.xml" />\n'
    content = content.replace('<compiler angle="radian" />', '<compiler angle="radian" />' + include_str)

    # Note: MuJoCo will merge <asset> and append <worldbody> children automatically!

    # 4. Save to phase4 world
    with open(phase4_world_path, "w", encoding="utf-8") as f:
        f.write(content)

    print(f"Created {phase4_world_path} successfully!")

if __name__ == "__main__":
    main()
