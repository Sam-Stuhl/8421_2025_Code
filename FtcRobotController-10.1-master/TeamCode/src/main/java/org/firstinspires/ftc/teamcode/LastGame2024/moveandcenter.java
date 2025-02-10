package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue Rectangle Tracking and Parking", group = "Autonomous")
public class moveandcenter extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private OpenCvWebcam webcam;

    // Constants for detecting the blue color
    private static final Scalar LOWER_BLUE = new Scalar(100, 150, 0); // HSV lower bound for blue
    private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255); // HSV upper bound for blue
    private static final double AREA_THRESHOLD = 1000; // Min area to consider valid detection

    // Motor speed and turn parameters
    private static final double MOVE_SPEED = 0.4;
    private static final double TURN_SPEED = 0.3;
    private static final double STOP_SPEED = 0.0;

    // Frame size
    private static final int FRAME_WIDTH = 640;
    private static final int FRAME_HEIGHT = 480;
    private static final int CENTER_TOLERANCE = 30; // Pixel tolerance to consider the robot centered

    @Override
    public void runOpMode() {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotor.class, "lFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "lBack");
        rightMotor = hardwareMap.get(DcMotor.class, "rFront");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rBack");

        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set up the pipeline for processing the camera frames
        BlueRectanglePipeline pipeline = new BlueRectanglePipeline();
        webcam.setPipeline(pipeline);

        // Start the camera stream
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        // Repeat the process of finding the blue rectangle and moving towards it 3 times
        for (int i = 0; i < 10000; i++) {
            moveTowardsRectangle(pipeline);
        }

        // Park the robot after centering on the last blue rectangle
        stopMotors();
    }

    // Move towards the nearest detected blue rectangle until the robot is centered on it
    private void moveTowardsRectangle(BlueRectanglePipeline pipeline) {
        while (opModeIsActive()) {
            Rect blueRectangle = pipeline.getBlueRectangle();

            if (blueRectangle != null) {
                // Calculate the center of the blue rectangle and the frame
                double centerX = blueRectangle.x + (blueRectangle.width / 2.0);
                double frameCenterX = FRAME_WIDTH / 2.0;

                // Determine how much the robot is off-center
                double xOffset = centerX - frameCenterX;

                // If the robot is sufficiently centered on the blue rectangle, move forward
                if (Math.abs(xOffset) < CENTER_TOLERANCE) {
                    telemetry.addData("Status", "Centered, Moving Forward");
                    telemetry.update();
                    setMotorPowers(-MOVE_SPEED, -MOVE_SPEED, MOVE_SPEED, MOVE_SPEED);
                    // sleep(1000); // Move forward for 1 second
                    //stopMotors();
                    break; // Break out of the loop to move to the next step
                } else if (xOffset < 0) {
                    // If the blue rectangle is to the left, turn right
                    telemetry.addData("Status", "Turning Right");
                    telemetry.update();
                    setMotorPowers(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED);
                } else {
                    // If the blue rectangle is to the right, turn left
                    telemetry.addData("Status", "Turning Left");
                    telemetry.update();
                    setMotorPowers(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED);
                }
            } else {
                // If no rectangle is detected, stop motors and try again
                telemetry.addData("Status", "No Rectangle Detected");
                telemetry.update();
                stopMotors();
            }
        }
    }

    // Set motor powers for both left and right motors
    private void setMotorPowers(double leftPower, double leftBackPower, double rightPower, double rightBackPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    // Stop the motors
    private void stopMotors() {
        leftMotor.setPower(STOP_SPEED);
        rightMotor.setPower(STOP_SPEED);
        leftBackMotor.setPower(STOP_SPEED);
        rightBackMotor.setPower(STOP_SPEED);
    }

    // Custom pipeline for detecting blue rectangles using OpenCV
    class BlueRectanglePipeline extends OpenCvPipeline {
        private Rect blueRectangle;

        @Override
        public Mat processFrame(Mat input) {
            // Convert the frame to HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Create a mask for the blue color
            Mat mask = new Mat();
            Core.inRange(hsv, LOWER_BLUE, UPPER_BLUE, mask);

            // Find contours in the mask
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest contour
            Rect largestRect = null;
            double largestArea = AREA_THRESHOLD; // Minimum area threshold

            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double area = Imgproc.contourArea(contour);

                if (area > largestArea) {
                    largestArea = area;
                    largestRect = rect;
                }
            }

            // Draw a rectangle around the detected blue rectangle
            if (largestRect != null) {
                Imgproc.rectangle(input, largestRect.tl(), largestRect.br(), new Scalar(0, 255, 0), 2);
                blueRectangle = largestRect;
            } else {
                blueRectangle = null;
            }

            // Release resources
            hsv.release();
            mask.release();
            hierarchy.release();

            return input;
        }

        public Rect getBlueRectangle() {
            return blueRectangle;
        }
    }
}
