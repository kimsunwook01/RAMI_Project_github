import numpy as np
import mujoco
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
            
            # YOLO v8 추론 실행 (RGB 이미지를 직접 입력, 터미널 로그 비활성화)
            results = self.yolo_model(rgb_image, verbose=False)
            
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
