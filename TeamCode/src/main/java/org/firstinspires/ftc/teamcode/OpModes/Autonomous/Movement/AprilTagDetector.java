package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Movement;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

/**
 * This is a class to locate the robot position by using the April Tag detections
 */
public class AprilTagDetector {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    //assigns variables to be used in the entire program. not assigned to functions
    private VisionPortal visionPortal;

    private VectorF finalVector;

    private AprilTagLibrary aprilTagLibrary;

    /**
     * The constructor
     * @param hardwareMap map of the hardware devices connected to this robot
     * @param telemetry telemetry logger
     */
    public AprilTagDetector(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the April detector here
        // TODO: to be implemented later
    }

    /**
     * Get the current robot location through April tag detections
     * @return the current robot position and yaw (if successful) | null (otherwise)
     */
    public Vector2d getLocation() {
        // Implement the april tag detection and distance calculation here

        // TODO: to be implemented later
        return null;
    }

    // More private functions here

}
