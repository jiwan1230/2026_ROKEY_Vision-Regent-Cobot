# YOLO 모델 학습 이력 및 성능 정리

## 모델 계보 요약

```
yolov8n.pt (COCO pretrained)
    │
    ▼ 100장, 2 class, 74 epoch
[Model 1] Seed Model
    reagent_seed_yolov8n/weights/best.pt
    │
    ▼ 800장, 3 class, 150 epoch (hand class 추가, auto-labeling)
[Model 2] 1st Full Train
    rokey_1st_autolabel_finetune_3class/weights/best.pt
    = src/vision_pkg/weights/reagent_yolov8n.pt  ← 1차 배포
    │
    ▼ 188장 추가, 3 class, 100 epoch (CVAT 수동 보정)
[Model 3] Fine-tune
    rokey_2nd_extra188_cvat_fixed/weights/best.pt
    = src/vision_pkg/weights/best.pt  ← 현재 배포
```

---

## 모델별 상세 정보

### Model 1 — Seed Model (초기 시드 모델)

| 항목 | 내용 |
|------|------|
| 목적 | auto-labeling용 기준 모델 생성 |
| 학습 데이터 | train 88장 + val 11장 = **총 99장** |
| 클래스 | 2개: `cup`, `height` |
| Base 모델 | `yolov8n.pt` (COCO pretrained) |
| 학습 설정 | epochs=100, batch=16, imgsz=640 |
| 실제 학습 epoch | 74 epoch (early stop) |
| 모델 파일 | `reagent_yolo_seed_dataset/runs/detect/runs/reagent_seed_yolov8n/weights/best.pt` |

**Best 성능 (epoch 36, mAP50-95 기준):**

| Precision | Recall | mAP50 | mAP50-95 |
|-----------|--------|-------|----------|
| 0.993 | 0.983 | **0.995** | **0.847** |

---

### Model 2 — 1st Full Train (800장 auto-labeling)

| 항목 | 내용 |
|------|------|
| 목적 | hand 클래스 추가, 대용량 데이터 학습 |
| 학습 데이터 | train 640장 + val 159장 = **총 799장** |
| 클래스 | 3개: `cup`, `height`, `hand` ← hand 추가 |
| Base 모델 | Model 1 best.pt (Seed 모델에서 fine-tune) |
| 학습 설정 | epochs=150, batch=16, imgsz=640 |
| 실제 학습 epoch | 149 epoch |
| 모델 파일 | `rokey_1st_autolabel_finetune_3class/weights/best.pt` → `weights/reagent_yolov8n.pt` |

**Best 성능 (epoch 140, mAP50-95 기준):**

| Precision | Recall | mAP50 | mAP50-95 |
|-----------|--------|-------|----------|
| 0.984 | 0.975 | **0.987** | **0.918** |

> Model 1 대비 mAP50-95 **+0.071** 향상 (0.847 → 0.918)
> hand 클래스가 추가되면서 mAP50이 소폭 감소 (3-class 평균으로 희석)

---

### Model 3 — Fine-tune (188장 CVAT 수동 보정)

| 항목 | 내용 |
|------|------|
| 목적 | 오검출 보정, 정밀도 향상 |
| 학습 데이터 | train 169장 + val 19장 = **총 188장** (추가 수동 라벨링) |
| 클래스 | 3개: `cup`, `height`, `hand` |
| Base 모델 | Model 2 (`reagent_yolov8n.pt`) |
| 학습 설정 | epochs=100, batch=16, imgsz=640 |
| 실제 학습 epoch | 100 epoch |
| 모델 파일 | `rokey_2nd_extra188_cvat_fixed/weights/best.pt` → `weights/best.pt` **← 현재 사용** |

**Best 성능 (epoch 94, mAP50-95 기준):**

| Precision | Recall | mAP50 | mAP50-95 |
|-----------|--------|-------|----------|
| 0.992 | 0.986 | **0.995** | **0.948** |

> Model 2 대비 mAP50-95 **+0.030** 추가 향상 (0.918 → 0.948)

---

## 전체 성능 비교

| 지표 | Model 1 (99장) | Model 2 (799장) | Model 3 (188장 추가) | 변화 |
|------|---------------|----------------|---------------------|------|
| Precision | 0.993 | 0.984 | **0.992** | ↑ |
| Recall | 0.983 | 0.975 | **0.986** | ↑ |
| **mAP50** | 0.995 | 0.987 | **0.995** | → (회복) |
| **mAP50-95** | 0.847 | 0.918 | **0.948** | ↑↑ |
| Classes | 2 | 3 | 3 | — |
| Train 데이터 | 88 | 640 | 169 (추가) | — |

```
mAP50-95 변화:
0.847 ──────────────────► 0.918 ──────────► 0.948
  Model 1                  Model 2            Model 3
  (99장, 2class)           (799장, 3class)    (+188장 fine-tune)
  +8.4%                                     +3.3%
  (전체 +12.0% 향상)
```

---

## 데이터셋 구성 흐름

```
[Model 1용 데이터]
 99장 직접 촬영 (cup, height 수동 라벨링)
      │
      ▼ auto-labeling으로 확장
[Model 2용 데이터]
 799장 (Model 1로 auto-label → 검수)
 + hand 클래스 추가
      │
      ▼ 오검출 케이스 CVAT로 수동 보정
[Model 3용 추가 데이터]
 188장 (문제 케이스 위주 수집, CVAT 수동 라벨링)
```

---

## 파일 위치 참조

| 모델 | 학습 결과 경로 | 배포 경로 |
|------|--------------|-----------|
| Model 1 | `~/webcam_yolo_project/autolabelling/reagent_yolo_seed_dataset/runs/detect/runs/reagent_seed_yolov8n/` | — |
| Model 2 | `~/webcam_yolo_project/autolabelling/reagent_full_train/rokey_1st_autolabel_finetune_3class/` | `src/vision_pkg/weights/reagent_yolov8n.pt` |
| Model 3 | `~/webcam_yolo_project/autolabelling/reagent_full_train/rokey_2nd_extra188_cvat_fixed/` | `src/vision_pkg/weights/best.pt` ✅ |
