# Pollen detector (v2)

A one-class object detector (`pollen`) for the Limelight neural detector pipeline,
trained for the FTC 2026-27 BIOBUZZ yellow pollen.

| | |
|---|---|
| Class | `pollen` |
| `limelight_neural_detector_8bit.tflite` | Limelight 3A (Detector Runtime: CPU) |
| `limelight_neural_detector_coral.tflite` | Limelights with a Google Coral (untested: we only have a Limelight 3A) |
| `limelight_neural_detector_labels.txt` | Label file for both models |
| Trained with | [Limelight Neural Network Trainer](https://tools.limelightvision.io/neural-network-trainer): platform Coral / CPU, variant Default, 20,000 steps, batch 16, INT8 quantized |
| Final validation loss | 0.055 (see `training_loss_graph.png`) |

## Training data

843 labeled source images, 2,137 after augmentation (train 1,941 / valid 162 / test 34).

1. [Biobuzz Pollen](https://universe.roboflow.com/junipers-workspace/biobuzz-pollen) by
   Junipers Workspace (CC BY 4.0): 638 images, class `pollen`.
2. [FTC BIOBUZZ Grounded Game Pieces](https://universe.roboflow.com/calin-cspro/ftc-biobuzz-grounded-game-pieces)
   by Calins Workspace (CC BY 4.0): balls on the floor at close range. We kept only the
   `pollen` boxes (704 boxes on 159 images), used 46 images with no pollen as negative
   examples, and left out 21 very small crops.

Preprocessing: auto-orient, resize to 640x640 (stretch). Augmentation, 3 outputs per image:
horizontal flip, brightness ±15%, exposure ±10%, blur up to 0.5 px.

## How well it works

Measured on our robot: Limelight 3A (firmware 2025.1), FTC SDK 12.0, REV Control Hub.

- Speed: about 11 FPS with the CPU runtime (the camera's CPU sits near 95%).
- Two balls 18-20 in away on a wood floor, exposure 20 ms: confidence 0.80-0.85 and 0.59-0.77.
  The same scene at 6 ms exposure: about 0.45. **Exposure is the biggest lever.**
- On a practice field (grey tiles, red field element, windows behind): 0.74-0.83 at close
  and mid range, and 0.64-0.84 for balls roughly 5-8 ft away (distance estimated from box
  size). A ball half hidden behind another scored 0.42-0.51.
- No false detections on tiles, field elements or the background in our tests.
- An earlier 4,000-step model trained on dataset 1 only scored about the same as this one
  at 6 ms exposure. Raising exposure improved results more than the extra training did.

## Limitations

- Trained on yellow pollen only. Nectar is not a class.
- No images from our own camera yet; adding field photos from your own robot should
  improve it further.
- Limelight 4 uses a Hailo accelerator and needs a `.hef` model. Train one from the same
  data with the platform set to Hailo-8L or Hailo-8.
