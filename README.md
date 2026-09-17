SPF 26-27 Prog!!!!!!

WERE GOING TO WORLDS

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
