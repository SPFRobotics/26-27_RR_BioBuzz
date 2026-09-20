SPF 26-27 Prog!!!!!!

WERE GOING TO WORLDS

## Robot code

Mechanisms use FTC SDK hardware, regular constructors, and direct method calls:

```java
// In init():
intake = new Intake(hardwareMap);
flywheel = new Flywheel(hardwareMap);

// In loop():
intake.intake();                 // Runs until outtake(), runAt(), or stop().
flywheel.runAtDistance(distanceInInches); // Or runAtRPM(rpm).
flywheel.update();               // Call every loop to maintain the target speed.

// In stop():
intake.stop();
flywheel.stop();
```

The default hardware names are `intakeMotor` and `OuttakeMotor`. Both constructors
also accept a custom name as their second argument. In TeleOp, gamepad 1 X toggles
the intake and gamepad 2 Square toggles the flywheel; holding a button does not
toggle repeatedly. The flywheel updates its distance target every loop while enabled.

Pedro Pathing handles driving/localization, and Dashboard exposes flywheel tuning.
The target pose and distance-to-RPM table are still placeholders and need robot tuning.

## Running with Sloth

The project includes shared Android Studio run configurations:

- **TeamCode**: installs the full Robot Controller app. Before building, it runs
  `:TeamCode:removeSlothRemote` to clear previous Sloth uploads from the connected robot.
- **Sloth Load**: runs `:TeamCode:deploySloth` to build and upload TeamCode changes
  without reinstalling the entire app.

Connect the robot and run **TeamCode** once to install Sloth on it. Then select
**Sloth Load** in the run configuration dropdown and press Run for normal code changes.
If the configurations do not appear immediately, reopen the project.

Use **TeamCode** again after adding or changing libraries, Android resources,
code outside `org.firstinspires.ftc.teamcode`, or `@Pinned` classes/annotations.
Sloth does not replace the Driver Station's Init/Start controls for running an OpMode.

See the [official Sloth setup instructions](https://github.com/Dairy-Foundation/Sloth#gradle-tasks).
