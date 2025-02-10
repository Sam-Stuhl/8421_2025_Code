package org.firstinspires.ftc.teamcode.RoadRunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "RRCAM", group = "Autonomous")
public class RRCAM extends LinearOpMode {

    public DcMotor intake = null;



    public DcMotor extendo = null;
    public Servo launcher = null;

    public Servo bucket = null;

    public Servo bucketarm = null;

    private Servo bucketR = null;

    private Servo bucketL = null;

    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera webcam;
    private BlueObjectPositionPipeline blueObjectPipeline;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(11.38, 66.97, Math.toRadians(267.71)))
                .splineTo(new Vector2d(13.06, 27.31), Math.toRadians(270.00))
                .splineTo(new Vector2d(47.14, 32.75), Math.toRadians(-1.59))
                .splineTo(new Vector2d(29.12, 32.75), Math.toRadians(180.00))
                .splineTo(new Vector2d(33.45, 56.78), Math.toRadians(5.71))
                .splineTo(new Vector2d(60.13, 61.39), Math.toRadians(5.91))
                .build();

        telemetry.setMsTransmissionInterval(50);


        intake = hardwareMap.get(DcMotor.class, "intake");
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        bucket = hardwareMap.get(Servo.class, "bucket");
        bucketarm = hardwareMap.get(Servo.class, "bucketarm");
        bucketR = hardwareMap.get(Servo.class, "bucketR");
        bucketL = hardwareMap.get(Servo.class, "bucketL");
        // rightEncoder =

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        intake.setDirection(DcMotor.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.FORWARD);
        //rightEncoder.setD


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Specify the camera name and connection type
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the OpenCV pipeline
        blueObjectPipeline = new BlueObjectPositionPipeline();

        // Initialize the camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

        // Set the pipeline
        webcam.setPipeline(blueObjectPipeline);

        // Start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Perform actions based on the object position
            switch (blueObjectPipeline.getObjectPosition()) {
                case CENTER:
                    // Move the robot left
                    drive.followTrajectorySequence(untitled0);
                    telemetry.addData("Status", "Center");    //
                    telemetry.update();
                    break;
                case RIGHT:
                    // Move the robot right
                    moveRight();
                    telemetry.addData("Status", "Right");    //
                    telemetry.update();
                    break;
                case UNKNOWN:
                    // Do nothing or implement default behavior
                    stopRobot();
                    telemetry.addData("Status", "Left");    //
                    telemetry.update();
                    break;
            }

            telemetry.addData("Object Position", blueObjectPipeline.getObjectPosition());
            telemetry.update();
        }

        // Stop streaming and close the camera
        webcam.stopStreaming();
        webcam.closeCameraDevice();

    }

    private void moveCenter() {

    }
    private void moveRight() {

    }
    private void stopRobot() {

    }

    public static class BlueObjectPositionPipeline extends OpenCvPipeline {

        private Mat hsvImage;
        private Mat mask;
        private Mat hierarchy;

        private enum ObjectPosition {
            CENTER, RIGHT, UNKNOWN
        }

        private ObjectPosition objectPosition = ObjectPosition.UNKNOWN;

        public BlueObjectPositionPipeline() {
            hsvImage = new Mat();
            mask = new Mat();
            hierarchy = new Mat();
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert the image to HSV color space
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            // Define the blue color range in HSV
            Scalar lowerBlue = new Scalar(90, 100, 100);
            Scalar upperBlue = new Scalar(120, 255, 255);

            // Create a binary mask of the blue color
            Core.inRange(hsvImage, lowerBlue, upperBlue, mask);

            // Find contours in the binary mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the contour with the largest area
            double maxArea = 0;
            int maxAreaIndex = -1;
            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > maxArea) {
                    maxArea = area;
                    maxAreaIndex = i;
                }
            }

            // Determine object position based on contour center x-coordinate
            if (maxAreaIndex != -1) {
                Rect boundingRect = Imgproc.boundingRect(contours.get(maxAreaIndex));
                int centerX = boundingRect.x + boundingRect.width / 2;

                if (centerX < input.cols() / 2) {
                    objectPosition = ObjectPosition.CENTER;
                } else {
                    objectPosition = ObjectPosition.RIGHT;
                }

                // Draw the contour with the largest area on the original image
                Imgproc.drawContours(input, contours, maxAreaIndex, new Scalar(0, 255, 0), 2);
            } else {
                objectPosition = ObjectPosition.UNKNOWN;
            }

            return input;
        }

        public ObjectPosition getObjectPosition() {
            return objectPosition;
        }
    }
}