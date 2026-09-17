# Building a OneDrive checkout on Windows

For a checkout inside OneDrive, the root `build.gradle` puts generated build
files outside the synced folder, under:

```text
%LOCALAPPDATA%\FTC\gradle-builds\<project-name>-<checkout-id>\
```

Each checkout gets its own folder. Module output is under `modules\TeamCode`
and `modules\FtcRobotController`. The debug app is at
`modules\TeamCode\outputs\apk\debug\TeamCode-debug.apk`.

This avoids the read-only folders and cloud file placeholders that have caused
`AccessDeniedException` and `Cannot snapshot ... not a regular file` failures.
Gradle's normal tracking of changed files remains enabled.

After updating the build settings, use **Sync Project with Gradle Files** in
Android Studio, then build or run as usual. Old `build` folders inside the
checkout are no longer used. Source files stay in the checkout.

Checkouts outside OneDrive and builds on other operating systems keep their
normal build folders. No machine-specific paths need to be committed.
