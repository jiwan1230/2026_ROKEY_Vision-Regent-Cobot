from ultralytics import YOLO
from pathlib import Path
import os

# =========================
# 경로 설정
# =========================
MODEL_PATH = Path.home() / "Downloads" / "reagent_yolov8n.onnx"
IMAGE_DIR = Path.home() / "Vision-Reagent-Cobot" / "src" / "vision_pkg" / "sample_images"
SAVE_DIR = Path.home() / "Vision-Reagent-Cobot" / "onnx_test_results"

# =========================
# 경로 확인
# =========================
if not MODEL_PATH.exists():
    raise FileNotFoundError(f"ONNX 모델을 찾을 수 없습니다: {MODEL_PATH}")

if not IMAGE_DIR.exists():
    raise FileNotFoundError(f"이미지 폴더를 찾을 수 없습니다: {IMAGE_DIR}")

# =========================
# 모델 로드
# =========================
model = YOLO(str(MODEL_PATH), task="detect")

print("[INFO] ONNX model loaded")
print(f"[INFO] Model path: {MODEL_PATH}")
print(f"[INFO] Image dir : {IMAGE_DIR}")

# =========================
# 이미지 폴더 추론
# =========================
results = model.predict(
    source=str(IMAGE_DIR),
    imgsz=640,
    conf=0.25,
    iou=0.45,
    device="cpu",
    save=True,
    project=str(SAVE_DIR),
    name="predict",
    exist_ok=True
)

# =========================
# 결과 출력
# =========================
print("\n========== Detection Results ==========")

for result in results:
    image_name = Path(result.path).name
    boxes = result.boxes

    print(f"\n[IMAGE] {image_name}")

    if boxes is None or len(boxes) == 0:
        print("  No detection")
        continue

    for i, box in enumerate(boxes):
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        xyxy = box.xyxy[0].tolist()

        class_name = model.names.get(cls_id, str(cls_id))

        print(
            f"  #{i} "
            f"class={class_name}({cls_id}), "
            f"conf={conf:.3f}, "
            f"xyxy=[{xyxy[0]:.1f}, {xyxy[1]:.1f}, {xyxy[2]:.1f}, {xyxy[3]:.1f}]"
        )

print("\n[INFO] Annotated images saved to:")
print(SAVE_DIR / "predict")
