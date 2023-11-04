package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Movement;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutonomousOpMode", group = "Autonomous") // Without this, this file will not show in the Autonomous section of the REV Driver Hub.
@Config
// Note that REV Driver Hub and REV Driver Station are synonymous.
public class AutonomousOpMode extends LinearOpMode {

    // Defines 4 Mecanum Wheel Motors, and then imu, which is set to IMU.
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    IMU imu;
    RobotMover robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declares motors using ID's that match the configuration on the REV Control Hub.
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Front Left Motor.
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Back Left Motor.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Front Right Motor.
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // Back Right Motor.

        // Set the zero power behavior to BRAKE for all motors.
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets all motors to use encoders.
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse the right side motors since we are using mecanum wheels.
        // Reverse left motors if you are using NeveRests.
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Declares IMU for the BHI260 IMU, which is on newer Control Hubs.
         * The older BNO055 IMU should not be used on newer Control Hubs.
         *
         * IMPORTANT:
         * For this code, the Control Hub needs to be mounted on one of the three orthogonal (right angle) planes
         * (X/Y, X/Z or Y/Z) and that the Hub has only been rotated in a range of 90 degree increments.
         * If the Control Hub is mounted on a surface angled at some non-90 Degree multiple (like 45 degrees) look at
         * the alternative SensorImuNonOrthogonal sample in:
         * FtcRobotController > src > main > java > org > firstinspires > ftc > robotcontroller > external > samples > SensorSensorImuNonOrthogonal
         *
         * This "Orthogonal" requirement means that:
         *
         * 1) The Logo printed on the top of the Hub can ONLY be pointing in one of six directions:
         *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
         *
         * 2) The USB ports can only be pointing in one of the same six directions:<br>
         *    FORWARD, BACKWARD, UP, DOWN, LEFT and RIGHT.
         *
         */

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(

                /*
                 * Assumes Control Hub orientation is in the Default Orientation, which
                 * is when the Control Hub is mounted horizontally with the printed logo
                 * pointing UP and the USB port pointing FORWARD.
                 */

                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot)); // Initializes IMU with the given robot orientation (UP, FORWARD).

        robot = new RobotMover(hardwareMap, telemetry, new AprilTagDetector(hardwareMap,telemetry),new Vector2d(0,0) );

        // Wait for the start button to be pressed on the Driver Station.
        waitForStart();

        imu.resetYaw();

        // Resets all motor encoders.
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Autonomous Code goes here:
        encoderMove(0.0,12.0,0.25);
        encoderMove(180.0,12.0,0.25);
        encoderMove(90.0,6.0,0.5);
        encoderMove(-90.0,6.0,0.5);
        //telemetry.addData("data", getCoordinates());

        robot.forwardTo(30);
        robot.turn(-90,-0.2,2.0);
        robot.forwardTo(2);

        telemetry.update();
    }

    private void encoderMove(double angle, double distanceInInches, double speed) {
        // Resets all motor encoders.
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Move the robot in the specified direction by converting angle to the corresponding velocities.
        double forward = Math.cos(Math.toRadians(angle));
        double sideways = Math.sin(Math.toRadians(angle));

        // Calculate motor powers using mecanum drive kinematics.
        double frontLeftPower = (forward + sideways) * speed;
        double frontRightPower = (forward - sideways) * speed;
        double backLeftPower = (forward - sideways) * speed;
        double backRightPower = (forward + sideways) * speed;

        /*
         * Calculate the encoder ticks by converting distanceInInches to encoderTicks.
         * This assumes that the 4 drive wheel motors are:
         * goBILDA 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX™ Shaft, 312 RPM, 3.3 - 5V Encoder).
         * The important part is that encoder resolution formula for this motor is: ((((1+(46/17))) * (1+(46/11))) * 28), which
         * equals an encoder resolution of 537.7 PPR at the Output Shaft.
         * The other important part is that we are using wheels with a diameter of 96mm.
         */
        double encoderTicks = (543.18 / 12) * distanceInInches;

        // Calculate the average encoder position based on the direction of motor movement.
        // Because wheels moving backwards have a positive encoder tick position:
        double positiveMotorPosition = 0;
        int positiveCount = 0;

        if (frontLeftPower < 0) {
            positiveMotorPosition += motorFrontLeft.getCurrentPosition();
            positiveCount += 1;
        }
        if (backLeftPower < 0) {
            positiveMotorPosition += motorBackLeft.getCurrentPosition();
            positiveCount += 1;
        }
        if (frontRightPower < 0) {
            positiveMotorPosition += motorFrontRight.getCurrentPosition();
            positiveCount += 1;
        }
        if (backRightPower < 0) {
            positiveMotorPosition += motorBackRight.getCurrentPosition();
            positiveCount += 1;
        }

        // Because wheels moving forwards have a negative encoder tick position:
        double negativeMotorPosition = 0;
        int negativeCount = 0;

        if (frontLeftPower > 0) {
            negativeMotorPosition += motorFrontLeft.getCurrentPosition();
            negativeCount += 1;
        }
        if (backLeftPower > 0) {
            negativeMotorPosition += motorBackLeft.getCurrentPosition();
            negativeCount += 1;
        }
        if (frontRightPower > 0) {
            negativeMotorPosition += motorFrontRight.getCurrentPosition();
            negativeCount += 1;
        }
        if (backRightPower > 0) {
            negativeMotorPosition += motorBackRight.getCurrentPosition();
            negativeCount += 1;
        }

        double averagePositiveMotorPosition = encoderTicks;
        double averageNegativeMotorPosition = encoderTicks;

        if (positiveCount > 0) {
            averagePositiveMotorPosition = positiveMotorPosition / positiveCount;
        }

        if (negativeCount > 0){
            averageNegativeMotorPosition = negativeMotorPosition / negativeCount;
        }

        while (opModeIsActive() && !((averageNegativeMotorPosition <= (encoderTicks + 10)) && (averageNegativeMotorPosition >= (encoderTicks - 10))) &&
                !((averagePositiveMotorPosition <= (encoderTicks + 10)) && (averagePositiveMotorPosition >= (encoderTicks - 10))) ) {

            // Calculate new averageMotorPosition if there are motors that have a positive encoder count:
            if (positiveCount > 0) {
                positiveMotorPosition = 0;

                if (frontLeftPower < 0) {
                    positiveMotorPosition += motorFrontLeft.getCurrentPosition();
                }
                if (backLeftPower < 0) {
                    positiveMotorPosition += motorBackLeft.getCurrentPosition();
                }
                if (frontRightPower < 0) {
                    positiveMotorPosition += motorFrontRight.getCurrentPosition();
                }
                if (backRightPower < 0) {
                    positiveMotorPosition += motorBackRight.getCurrentPosition();
                }
                averagePositiveMotorPosition = positiveMotorPosition / positiveCount;
            }

            // Calculate new averageMotorPosition if there are motors that have a positive encoder count:
            if (negativeCount > 0) {
                negativeMotorPosition = 0;

                if (frontLeftPower > 0) {
                    negativeMotorPosition += motorFrontLeft.getCurrentPosition();
                }
                if (backLeftPower < 0) {
                    negativeMotorPosition += motorBackLeft.getCurrentPosition();
                }
                if (frontRightPower > 0) {
                    negativeMotorPosition += motorFrontRight.getCurrentPosition();
                }
                if (backRightPower > 0) {
                    negativeMotorPosition += motorBackRight.getCurrentPosition();
                }
                averageNegativeMotorPosition = negativeMotorPosition / negativeCount;
            }

            // Set motor powers.
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            // Add telemetry data.
            telemetry.addData("motorFrontLeft Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorBackLeft Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight Position", motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackRight Position", motorBackRight.getCurrentPosition());

            telemetry.addData("averagePositiveMotorPosition", averagePositiveMotorPosition);
            telemetry.addData("averageNegativeMotorPosition", averageNegativeMotorPosition);
            telemetry.addData("encoderTicks", encoderTicks);

            telemetry.update();
        }
        // Stop the robot once the movement is complete.
        robot.stopRobot();
        telemetry.addData("Movement:", "Complete");
        telemetry.update();
    }
}