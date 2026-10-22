# TeleOp driver guide

Current visual: [PS4 DualShock 4 controller guide](teleop-driver-controls-ps4.png).

Based on the current working copy of `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TeleOp/Tele.java`, reviewed October 22, 2026. Select the **TeleOp** OpMode. `OmSanityTele` has different bindings.

| Controller | Control (PS4) | Action |
| --- | --- | --- |
| Driver 1 | Left stick | Drive forward/back and strafe; robot-relative |
| Driver 1 | Right stick, left/right | Turn |
| Driver 1 | L2, hold | Scale drive and turn to 35%; activates above 10% trigger travel |
| Driver 1 | Square, tap | Toggle intake on/off |
| Driver 1 | R1, hold | Reverse intake and transfer; open blocker |
| Driver 1 | Cross, tap | Toggle pushback ramp down/up |
| Driver 2 | L1, tap | Toggle both flywheels on/off |
| Driver 2 | Triangle, tap | Toggle transfer on/off |
| Driver 2 | Circle, tap | Toggle blocker open/closed |

Tap once to change a toggle; release before tapping again. Hold controls are temporary.

On start: intake, transfer and flywheels off; blocker closed; ramp up. Driver 1's left trigger changes translation and rotation together. No drive-mode toggle is bound; the current mode is robot-relative.

Reverse temporarily overrides intake, transfer and blocker outputs. Their saved toggle states resume on release. Their toggle inputs are ignored both while reverse is held and in its release loop, so release R1 before tapping Square, Triangle or Circle. Flywheel and ramp toggles still work during reverse. Reverse does not stop the flywheels.

Flywheel targets when enabled are Pollen 2700 RPM and Nectar 3300 RPM; the code also requires a finite, nonnegative target distance. The poster focuses on driver actions rather than tuning values.

The PS4 poster was edited with the built-in image generation tool. The exact edit prompts are saved in `teleop-ps4-visual-prompt.txt`. Update the poster whenever these bindings change. The earlier mixed-controller poster and its prompt are retained as prior versions.
