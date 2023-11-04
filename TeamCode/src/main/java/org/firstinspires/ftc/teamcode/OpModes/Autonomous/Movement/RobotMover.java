package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Movement;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.commands.ImuCommands;
import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.ImuAngularVelocityResponse;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.concurrent.TimeUnit;


/**
 * This class contains all methods related to moving robots
 */
public class RobotMover  {

    HardwareMap hardwareMap; //< hardware map
    Telemetry telemetry; //< telemetry logger
    AprilTagDetector aprilTagDetector; //< the april tag detector
    Vector2d lastPosition; //< the last known position of the robot

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    IMU imu;

    /**
     * the constructor
     *
     * @param _hardwareMap      map of the hardware devices connected to this robot
     * @param _telemetry        telemetry logger
     * @param _aprilTagDetector the April Tag detector
     * @param _startPos         the robot start position
     */
    public RobotMover(HardwareMap _hardwareMap, Telemetry _telemetry, AprilTagDetector _aprilTagDetector, Vector2d _startPos) {
        // Initialize the robot here
        // 1. Initialize the motors and set the encoder mode
        // 2. Save the telemetry, aprilTagDetector and start position

        hardwareMap = _hardwareMap;
        telemetry = _telemetry;
        aprilTagDetector = _aprilTagDetector;
        lastPosition = _startPos;
        //Do motor and IMU init here


        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // Front Left Motor.
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); // Back Left Motor.
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // Front Right Motor.
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");


        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu"); // Retrieves the IMU from the hardware map.

        // Adjusts the orientation parameters to match the robot (note that IMU is set to imu).
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized"); // Adds Initialized Status.
        telemetry.update();
    }
    public void turn(double targetYaw, double speed, double allowableError) {



        /*
         * The yaw value of the robot INCREASES as the robot is rotated Counter Clockwise.
         * The Control Hub's yaw value ranges from 0 to 180 degrees counterclockwise, and
         * 180 to 0 degrees clockwise.
         *
         * The speed and targetYaw are negated here because that effectively reverses the Control Hub's
         * yaw range, making it now range from 0 to 180 degrees counterclockwise, and
         * 180 to 0 degrees clockwise.
         */

        double targetYawDegrees = -targetYaw;
        double power = -speed;

        // Define the allowable error for stopping the turn.

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Calculate the initial difference between the target yaw and the current yaw.
        double yawDifference = targetYawDegrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // Determine the direction of the turn based on the sign of the power.
        double direction = Math.signum(power);
        int a= 1;

        // Continue turning until the robot reaches the target yaw within the allowable error.
        while (Math.abs(yawDifference) > allowableError ) {
            // Recalculate the yaw difference in each iteration.
            yawDifference = targetYawDegrees - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Calculate the turn power based on the yaw difference and the specified speed.
            double turnPower = Math.min(Math.abs(yawDifference / 10), Math.abs(power)) * direction;

            // Set motor powers to achieve the turn.
            motorFrontLeft.setPower(turnPower);
            motorBackLeft.setPower(turnPower);
            motorFrontRight.setPower(-turnPower);
            motorBackRight.setPower(-turnPower);

            /*
             * Adds Telemetry data for monitoring.
             * Target yaw angle,
             * Yaw (Z) velocity (in Def/Sec),
             * Current yaw angle.
             */
            a+=1;
            telemetry.addData("Target Yaw", targetYawDegrees);
            telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Current Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("count",a);
            telemetry.update();
        }

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
//        // Stop the robot once the turn is complete.
//        stopRobot();
    }

    public boolean stopRobot() {
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        return false;
    }

    /**
     * get the current robot orientation in the FTC field coordinate
     *
     * @return the orientation angle in the FTC field cooridnate system
     */
    public double getCurrentOrientation() {
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        double Yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        return Yaw;
    }
    public void resetEncoder(){
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }/**
     * get the current robot position in the FTC field coordinate
     *
     * @return the robot position
     */
    public Vector2d getCurrentPosition() {
        Vector2d pos = aprilTagDetector.getLocation();
        if (pos != null) {
            lastPosition = pos;
        }
        return lastPosition;
    }

    /**
     * turn the robot to the input orientation in the FTC field coordinate
     *
     * @param orientation the angle of the FTC field coordinate system
     * @return true (if successful) | false (otherwise)
     */
    public boolean turnTo(double orientation) {


        double angleToTurn = orientation - getCurrentOrientation();
        if (0 == angleToTurn) {
            return true;
        }


//        turn(angleToTurn);
        stopRobot();
        resetEncoder();
        return false;
    }

    /**
     * drive the robot forward to the input distance
     *
     * @param distance the driving distance
     * @return true (if successful) | false (otherwise)
     */
    public void forwardTo(double distance) {
        Vector2d start = getCurrentPosition();
        double angle = getCurrentOrientation();
        int encoderTicks = (int) ((543/12) * -distance);
        resetEncoder();
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorFrontLeft.setPower(0.5);
        motorBackLeft.setTargetPosition(encoderTicks);
        motorBackRight.setTargetPosition(encoderTicks);
        motorFrontRight.setTargetPosition(encoderTicks);
        motorFrontLeft.setTargetPosition(encoderTicks);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy() && motorBackRight.isBusy()) {
            assert true; // Passes.
            telemetry.addData("fl", motorFrontLeft.isBusy());
            telemetry.addData("bl",motorBackLeft.isBusy());
            telemetry.addData("fr", motorFrontRight.isBusy());
            telemetry.addData("br", motorBackRight.isBusy());
            telemetry.update();
        }
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
//        stopRobot();

        // update
        Vector2d delta = new Vector2d(distance, 0).rotated(angle);
        lastPosition = lastPosition.plus(delta);

    }

    public boolean backwardTo(double distance) {
        Vector2d start = getCurrentPosition();
        double angle = getCurrentOrientation();
        int encoderTicks = (int) ((543/12) * -distance);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(0.5);
        motorFrontRight.setPower(0.5);
        motorFrontLeft.setPower(0.5);
        motorBackLeft.setTargetPosition(-encoderTicks);
        motorBackRight.setTargetPosition(-encoderTicks);
        motorFrontRight.setTargetPosition(-encoderTicks);
        motorFrontLeft.setTargetPosition(-encoderTicks);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy()) {
            assert true; // Passes.
        }
        stopRobot();
        resetEncoder();
        // update
        Vector2d delta = new Vector2d(distance, 0).rotated(angle);
        lastPosition = lastPosition.plus(delta);

        return false;
    }
    /**
     * Move the robot along the input path
     *
     * @param path the transition points along the path
     * @return true (if successful) | false (otherwise)
    //     */
//    public boolean moveTo(Vector2d[] path) {
//        for (Vector2d nextStop : path) {
//            Vector2d currentPosition = getCurrentPosition();
//            double currentOrientation = getCurrentOrientation();
//            Vector2d delta = nextStop.minus(currentPosition);
//
//            if (!turn(delta.angle())) {
//                telemetry.addData("moveTo() Error", "Failed to turn to ", nextStop);
//                return false;
//            }
//
//            if (!forwardTo((int) currentPosition.distTo(nextStop))) {
//                telemetry.addData("moveTo() Error", "Failed to move to ", nextStop);
//                return false;
//            }
//        }
//
//        return true;
//    }
}
