import numpy as np
import mujoco
import cv2
from ultralytics import YOLO
from ultralytics import YOLO

class VisionProcessor:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        
        # YOLOv8 모델 초기화 (가장 빠르고 가벼운 Nano 모델 사용)
        self.yolo_model = YOLO('yolov8n.pt')
        
        # 렌더러 초기화 (일반적인 640x480 해상도 사용)
        self.width = 640
        self.height = 480
        self.renderer = mujoco.Renderer(self.model, self.height, self.width)
        
    def process_camera(self, data: mujoco.MjData, camera_name: str):
        """
        주어진 카메라에서 RGB 프레임을 렌더링하고 YOLO v8으로 객체를 탐지합니다.
        """
        try:
            # 렌더러의 시점을 해당 카메라로 업데이트
            self.renderer.update_scene(data, camera=camera_name)
            
            # RGB 이미지 추출 (H, W, 3) 
            rgb_image = self.renderer.render()
            
            # 디버깅용 캡처 이미지 저장 (첫 프레임만 또는 지속 저장)
            if camera_name == "head_camera":
                cv2.imwrite(r"C:\Users\sunny\.gemini\antigravity-ide\brain\27554dc4-80c9-47a9-a5e3-d6d1f0d2e6f8\camera_view.png", cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
            elif camera_name == "gripper_camera":
                cv2.imwrite(r"C:\Users\sunny\.gemini\antigravity-ide\brain\27554dc4-80c9-47a9-a5e3-d6d1f0d2e6f8\gripper_camera_view.png", cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
            
            # YOLO v8 추론 실행 (원색 도형 인식을 위해 confidence를 0.05로 대폭 낮춤)
            results = self.yolo_model(rgb_image, verbose=False, conf=0.05)
            
            detections = []
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # 박스 좌표 (x1, y1, x2, y2)
                    b = box.xyxy[0].tolist()
                    # 클래스 및 신뢰도
                    cls_id = int(box.cls[0].item())
                    conf = box.conf[0].item()
                    cls_name = self.yolo_model.names[cls_id]
                    
                    detections.append({
                        "class": cls_name,
                        "confidence": conf,
                        "bbox": [int(x) for x in b]
                    })
                    
            return detections
            
        except Exception as e:
            print(f"[{camera_name}] Vision processing error: {e}")
            return []
