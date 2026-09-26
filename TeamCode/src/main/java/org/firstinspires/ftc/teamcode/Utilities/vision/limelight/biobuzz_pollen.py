"""BIOBUZZ yellow pollen detector. Paste this file into a Python SnapScript.

Output v1: [found, count, x_normalized, y_normalized, diameter_percent,
            area_percent, circularity, version]. See README.md for the contract.
Only cv2 and numpy are required; both are provided by Limelight.
"""

import cv2
import numpy as np

# Starting values for YELLOW pollen, not measurements from your camera.
# OpenCV HSV: hue 0..179, saturation/brightness 0..255.
HSV_MIN = (18, 85, 65)
HSV_MAX = (40, 255, 255)

# Search rectangle as fractions of the image: left, top, right, bottom.
# Keep the full view initially: pollen may be on flowers as well as the floor.
ROI = (0.0, 0.0, 1.0, 1.0)
MIN_AREA_PERCENT = 0.03
MAX_AREA_PERCENT = 30.0
MIN_CIRCULARITY = 0.65
MIN_ASPECT_RATIO = 0.60       # Short bounding-box side / long side.
MIN_CIRCLE_FILL = 0.70        # Contour area / enclosing-circle area.
REJECT_EDGE_BLOBS = True     # Partial balls at the image/ROI edge are unreliable.
DRAW_PREVIEW = True
SHOW_MASK = False            # True shows exactly which pixels match yellow.
SCHEMA_VERSION = 1.0


def runPipeline(image, llrobot):
    """Called by Limelight once per frame; llrobot is reserved for future use."""
    height, width = image.shape[:2]
    output = [0.0] * 7 + [SCHEMA_VERSION]
    no_contour = np.array([[]], dtype=np.int32)
    left = int(round(ROI[0] * width))
    top = int(round(ROI[1] * height))
    right = int(round(ROI[2] * width))
    bottom = int(round(ROI[3] * height))
    if not (0 <= left < right <= width and 0 <= top < bottom <= height):
        cv2.putText(image, "Invalid ROI", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return no_contour, image, output

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_MIN, HSV_MAX)
    # Small closing operation fills tiny gaps. Large kernels can join nearby balls.
    kernel_size = max(3, int(round(min(width, height) / 240.0)) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # Zero everything outside ROI, keeping contours in full-image coordinates.
    mask[:top, :] = 0
    mask[bottom:, :] = 0
    mask[:, :left] = 0
    mask[:, right:] = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    image_area = float(width * height)

    for contour in contours:
        area = float(cv2.contourArea(contour))
        area_percent = 100.0 * area / image_area
        if not MIN_AREA_PERCENT <= area_percent <= MAX_AREA_PERCENT:
            continue
        x, y, box_width, box_height = cv2.boundingRect(contour)
        if REJECT_EDGE_BLOBS and (x <= left or y <= top or
                                  x + box_width >= right or
                                  y + box_height >= bottom):
            continue
        aspect = min(box_width, box_height) / float(max(box_width, box_height))
        if aspect < MIN_ASPECT_RATIO:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = min(1.0, 4.0 * np.pi * area / (perimeter * perimeter))
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        if radius <= 0:
            continue
        circle_fill = area / (np.pi * radius * radius)
        if circularity < MIN_CIRCULARITY or circle_fill < MIN_CIRCLE_FILL:
            continue
        candidates.append((contour, area_percent, cx, cy, radius, circularity))

    # Largest accepted blob first; then closest to image center; then leftmost.
    # This is an apparent-size policy, not proof of distance or persistent tracking.
    candidates.sort(key=lambda c: (-c[1], abs(c[2] - width / 2.0), c[2], c[3]))
    if SHOW_MASK:
        image = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    if DRAW_PREVIEW:
        cv2.rectangle(image, (left, top), (right - 1, bottom - 1), (180, 180, 180), 1)
        for candidate in candidates:
            cv2.drawContours(image, [candidate[0]], -1, (255, 180, 0), 1)

    if not candidates:
        if DRAW_PREVIEW:
            cv2.putText(image, "Pollen: none", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return no_contour, image, output

    selected, area_percent, cx, cy, radius, circularity = candidates[0]
    output = [1.0, float(len(candidates)),
              float((cx - width / 2.0) / (width / 2.0)),
              float((height / 2.0 - cy) / (height / 2.0)),
              float(200.0 * radius / width), float(area_percent),
              float(circularity), SCHEMA_VERSION]
    if DRAW_PREVIEW:
        center = (int(round(cx)), int(round(cy)))
        cv2.circle(image, center, int(round(radius)), (0, 255, 0), 2)
        cv2.drawMarker(image, center, (0, 255, 0), cv2.MARKER_CROSS, 12, 1)
        cv2.putText(image, "Pollen: %d  x:%+.2f y:%+.2f" %
                    (len(candidates), output[2], output[3]), (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    # Returning the selected contour also lets Limelight calculate tx/ty/ta.
    return selected, image, output
