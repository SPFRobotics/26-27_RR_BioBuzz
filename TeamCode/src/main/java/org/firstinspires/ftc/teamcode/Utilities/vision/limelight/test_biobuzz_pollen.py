"""Offline behavior checks using synthetic images, not camera validation.

Run: python -m unittest discover -s TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Utilities/vision/limelight -p "test_*.py" -v
"""
import unittest

import cv2
import numpy as np

import biobuzz_pollen as pipeline


class PollenPipelineTests(unittest.TestCase):
    def frame(self, width=320, height=240):
        return np.full((height, width, 3), 45, dtype=np.uint8)

    def run_image(self, image):
        contour, preview, values = pipeline.runPipeline(image, [])
        self.assertEqual(len(values), 8)
        self.assertTrue(all(np.isfinite(values)))
        self.assertEqual(values[7], 1.0)
        self.assertEqual(preview.shape, image.shape)
        return contour, values

    def test_blank_and_wrong_colors(self):
        image = self.frame()
        for x, color in [(60, (255, 0, 0)), (160, (0, 255, 0)), (260, (255, 0, 255))]:
            cv2.circle(image, (x, 120), 20, color, -1)
        contour, values = self.run_image(image)
        self.assertEqual(contour.size, 0)
        self.assertEqual(values[:7], [0.0] * 7)

    def test_centered_ball(self):
        image = self.frame()
        cv2.circle(image, (160, 120), 20, (0, 255, 255), -1)
        contour, values = self.run_image(image)
        self.assertGreater(contour.size, 0)
        self.assertEqual(values[:2], [1.0, 1.0])
        self.assertAlmostEqual(values[2], 0, places=3)
        self.assertAlmostEqual(values[3], 0, places=3)
        self.assertAlmostEqual(values[4], 12.5, delta=0.1)

    def test_largest_selection_count_and_axis_signs(self):
        image = self.frame()
        cv2.circle(image, (60, 180), 12, (0, 255, 255), -1)
        cv2.circle(image, (240, 60), 25, (0, 255, 255), -1)
        _, values = self.run_image(image)
        self.assertEqual(values[:2], [1.0, 2.0])
        self.assertAlmostEqual(values[2], 0.5, places=2)
        self.assertAlmostEqual(values[3], 0.5, places=2)

    def test_holes_inside_pollen(self):
        image = self.frame()
        cv2.circle(image, (160, 120), 25, (0, 200, 200), -1)
        for point in [(150, 115), (170, 115), (160, 132)]:
            cv2.circle(image, point, 4, (45, 45, 45), -1)
        _, values = self.run_image(image)
        self.assertEqual(values[:2], [1.0, 1.0])

    def test_noise_square_and_strip_rejected(self):
        image = self.frame()
        cv2.circle(image, (30, 30), 2, (0, 255, 255), -1)
        cv2.rectangle(image, (70, 80), (110, 120), (0, 255, 255), -1)
        cv2.rectangle(image, (160, 160), (280, 170), (0, 255, 255), -1)
        _, values = self.run_image(image)
        self.assertEqual(values[0], 0)

    def test_edge_ball_rejected(self):
        image = self.frame()
        cv2.circle(image, (4, 120), 25, (0, 255, 255), -1)
        _, values = self.run_image(image)
        self.assertEqual(values[0], 0)

    def test_no_previous_target_reused(self):
        image = self.frame()
        cv2.circle(image, (160, 120), 20, (0, 255, 255), -1)
        self.run_image(image)
        _, values = self.run_image(self.frame())
        self.assertEqual(values[:7], [0.0] * 7)

    def test_coordinates_at_two_resolutions(self):
        for scale in (1, 2):
            image = self.frame(320 * scale, 240 * scale)
            cv2.circle(image, (80 * scale, 180 * scale), 20 * scale, (0, 255, 255), -1)
            _, values = self.run_image(image)
            self.assertAlmostEqual(values[2], -0.5, places=2)
            self.assertAlmostEqual(values[3], -0.5, places=2)
            self.assertAlmostEqual(values[4], 12.5, delta=0.1)

    def test_roi_preserves_full_image_coordinates(self):
        original = pipeline.ROI
        try:
            pipeline.ROI = (0.5, 0.0, 1.0, 1.0)
            image = self.frame()
            cv2.circle(image, (80, 120), 25, (0, 255, 255), -1)
            cv2.circle(image, (240, 120), 20, (0, 255, 255), -1)
            _, values = self.run_image(image)
            self.assertEqual(values[1], 1)
            self.assertAlmostEqual(values[2], 0.5, places=2)
        finally:
            pipeline.ROI = original


if __name__ == "__main__":
    unittest.main()
