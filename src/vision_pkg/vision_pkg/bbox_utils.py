"""Helpers for mapping YOLOv8n 'cup'/'height'/'hand' detections onto tube
slots and a calibrated liquid height.

Calibration is derived empirically from the reagent dataset annotations:
the 'height' box spans from the liquid surface down to (approximately) the
bottom of the 'cup' box. The fraction of the cup's vertical extent covered
by the height box, measured from the bottom, is taken as the fill ratio.
"""
from dataclasses import dataclass


@dataclass
class Box:
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float

    @property
    def cx(self):
        return (self.x1 + self.x2) / 2.0

    @property
    def height(self):
        return self.y2 - self.y1

# 260624 jiwan rank 기반 재정렬(assign_tube_order)은 폐기로 컵 개수가 줄면
# 남은 컵들이 인덱스 0부터 다시 채워져서 tube_index가 밀리는 문제가 있었음.
# -> 슬롯 anchor(cx, y1) 기준 매칭으로 교체. anchor는 liquid_height_detector_node가
# 컵 3개가 동시에 보이는 시점에 부트스트랩해서 들고 있고, 여기서는 그 anchor에
# 가장 가까운 박스를 골라주는 순수 함수만 담당.
def match_cups_to_anchors(cup_boxes, slot_anchors, slot_x_tolerance_px, row_y_tolerance_px):
    """Match detected cup boxes to fixed slot anchors [(cx, y1), ...].

    A box only matches a slot if it's within slot_x_tolerance_px (x) AND
    row_y_tolerance_px (y) of that slot's anchor - the y check is what stops
    a back-row cup from being mistaken for a tube that's temporarily empty
    (e.g. mid-dispose) just because its x happens to line up with the anchor.

    Returns a list of length len(slot_anchors), each entry a Box or None.
    """
    num_tubes = len(slot_anchors)
    slots = [None] * num_tubes
    best_score = [None] * num_tubes

    for box in cup_boxes:
        for idx, (anchor_x, anchor_y) in enumerate(slot_anchors):
            dx = abs(box.cx - anchor_x)
            dy = abs(box.y1 - anchor_y)
            if dx > slot_x_tolerance_px or dy > row_y_tolerance_px:
                continue
            # 위치 우선: anchor에 가장 가까운 박스를 선택.
            # confidence 기반 score는 cup/height bbox가 비슷한 위치에 있을 때
            # 신뢰도 높은 엉뚱한 박스가 슬롯을 가로채는 문제가 있어 제거함.
            score = -(dx + dy)
            if best_score[idx] is None or score > best_score[idx]:
                best_score[idx] = score
                slots[idx] = box

    return slots

# end

def match_height_box(cup_box, height_boxes):
    """Pick the height box whose center x falls within the cup box and that
    has the largest vertical overlap with it."""
    best = None
    best_overlap = 0.0
    for hbox in height_boxes:
        if not (cup_box.x1 <= hbox.cx <= cup_box.x2):
            continue
        overlap = min(cup_box.y2, hbox.y2) - max(cup_box.y1, hbox.y1)
        if overlap > best_overlap:
            best_overlap = overlap
            best = hbox
    return best


def liquid_fill_fraction(cup_box, height_box):
    """Fraction (0..1) of the tube's vertical extent that is filled,
    measured from the bottom of the cup box up to the top of the height box.
    """
    cup_span = cup_box.y2 - cup_box.y1
    if cup_span <= 0:
        return 0.0
    fraction = (cup_box.y2 - height_box.y1) / cup_span
    return max(0.0, min(1.0, fraction))
