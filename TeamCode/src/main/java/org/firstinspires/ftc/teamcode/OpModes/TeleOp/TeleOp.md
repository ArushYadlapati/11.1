## TeleOp Code Modifications

#### This `.md` file is primarily to help drivers of the FTC team #23511, the [Seattle Solvers](https://www.seattlesolvers.com/home) understand the gamepad controls for our robot for 2023-24 FTC Season, CENTERSTAGE℠ presented by Raytheon Technologies.


It has been modified by adding the TeleOp code for [GoBilda's Strafer Chassis Kit](https://www.gobilda.com/strafer-chassis-kit-v5/), which uses a mecanum drivetrain. This code was originated from [Game Manual 0's Mecanum Drivetrain Turorial](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html), and then modified to our needs. All of the robot code is in [Java](https://www.java.com/en/).

- The robot will strafe/move faster based on how hard the Left Trigger (LT) is pushed/pressed (when not pressed, the default speed is `0.15` or `15%` speed, when fully pressed, the speed is increased to `1` or `100%`).

- Uses `RUN_USING_ENCODERS` and `ZeroPowerBehavior.BRAKE`.

- Modified all of the controller code to change it from Xbox controllers to our controller, the [Logitech F310](https://www.amazon.com/Logitech-940-000110-Gamepad-F310/dp/B003VAHYQY/ref=sr_1_1?keywords=logitech+f310&qid=1691515991&sr=8-1).

## Gamepad Mapping

#### Gamepad 1:

- Left joystick: Moving/Strafing
- Right joystick: Turning
- D-Pad:
  - A: Lift up arm
  - B: Close arm
  - X: None
  - Y: None
- Left bumper: None
- Right bumper: None
- Left trigger: Accelerate Robot Moving/Strafing
- Right trigger: None
- Start button: Reset yaw for field-centric
- Back button: Launch drone

## Download Process for Android Studio

- I recommend using [GitHub Desktop](https://desktop.github.com/), as it is very simple to use.
- If you don't want to download it, you can also [download the code](https://github.com/FTC-23511/SolversFTC-2023-24/archive/refs/heads/code.zip), and then extract it to your preferred folder (such as Downloads) since it is in `.zip`.
- From there, all you need to do is to open the extracted folder in Android Studio. The folder will be called `SolversFTC-2023-24`.
- MAKE SURE TO OPEN THE FOLDER INSIDE `SolversFTC-2023-24` (which is also called `SolversFTC-2023-2024`).

- The path name should look something like this (assuming you extracted the .zip folder to your Downloads folder on Windows): `C:\Users\user_name\Downloads\SolversFTC-2023-24-code\SolversFTC-2023-2024-code`

  - Make sure to replace user_name with your account's actual name of user

## Authors

- [@ArushYadlapati](https://www.github.com/ArushYadlapati)

- [@Solver4444](https://www.github.com/@Solver4444)

- The rest of this README.md is unmodified from the FTC's official SDK for the 2023-2024 season, CENTERSTAGE: [FtcRobotController v9.0](https://github.com/FIRST-Tech-Challenge/FtcRobotController#readme).
