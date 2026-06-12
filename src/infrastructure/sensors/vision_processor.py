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
        
        # QR 코드 감지기 초기화
        self.qr_detector = cv2.QRCodeDetector()
        
        # 렌더러 초기화 (일반적인 640x480 해상도 사용)
        self.width = 640
        self.height = 480
        self.renderer = mujoco.Renderer(self.model, self.height, self.width)
        
    def process_camera(self, data: mujoco.MjData, camera_name: str, detect_yolo: bool = True, detect_qr: bool = True):
        """
        주어진 카메라에서 RGB 프레임을 렌더링하고 객체(YOLO) 및 QR 코드를 탐지합니다.
        """
        try:
            # 렌더러의 시점을 해당 카메라로 업데이트
            self.renderer.update_scene(data, camera=camera_name)
            
            # RGB 이미지 추출 (H, W, 3) 
            rgb_image = self.renderer.render()
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
            detections = []
            
            # YOLO v8 추론
            if detect_yolo:
                results = self.yolo_model(rgb_image, verbose=False, conf=0.05)
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        b = box.xyxy[0].tolist()
                        cls_id = int(box.cls[0].item())
                        conf = box.conf[0].item()
                        cls_name = self.yolo_model.names[cls_id]
                        
                        detections.append({
                            "type": "yolo",
                            "class": cls_name,
                            "confidence": conf,
                            "bbox": [int(x) for x in b]
                        })
                        
            # QR 코드 탐지
            if detect_qr:
                retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(bgr_image)
                if retval and points is not None:
                    for idx, pts in enumerate(points):
                        pts = pts.astype(int)
                        x_min, y_min = np.min(pts, axis=0)
                        x_max, y_max = np.max(pts, axis=0)
                        cx = int((x_min + x_max) / 2)
                        cy = int((y_min + y_max) / 2)
                        info = decoded_info[idx] if decoded_info else ""
                        
                        detections.append({
                            "type": "qr",
                            "class": "switch_qr",
                            "data": info,
                            "confidence": 1.0,
                            "bbox": [int(x_min), int(y_min), int(x_max), int(y_max)],
                            "center": [cx, cy],
                            "points": pts.tolist()
                        })
                        
            return detections, bgr_image
            
        except Exception as e:
            print(f"[{camera_name}] Vision processing error: {e}")
            return [], None
