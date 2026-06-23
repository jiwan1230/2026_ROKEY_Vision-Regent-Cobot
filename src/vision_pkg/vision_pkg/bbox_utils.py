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


def assign_tube_zones(cup_boxes, num_tubes, frame_width):
    """Bucket cup boxes into `num_tubes` equal-width left-to-right zones.

    Returns a list of length num_tubes, each entry either a Box or None
    (zone has no detected cup). Keeps tube indices stable (index 0 = left)
    even if a tube briefly fails to detect, instead of relying on rank order.
    """
    zone_width = frame_width / num_tubes
    slots = [None] * num_tubes

    for box in cup_boxes:
        zone = int(box.cx // zone_width)
        zone = max(0, min(num_tubes - 1, zone))
        current = slots[zone]
        if current is None or box.conf > current.conf:
            slots[zone] = box

    return slots


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
