package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "MoveForward20Inches", group = "Robot")
public class Encoder extends LinearOpMode {

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    public DcMotor intake = null;

    public DcMotor extendo = null;
    public Servo launcher = null;

    public Servo bucket = null;

    public Servo bucketarm = null;

    private Servo bucketR = null;

    private Servo bucketL = null;

    private VideoCapture capture;

    public static final double LAUNCHER = 1;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.6;

    double encoderRight = 0.0;
    double encoderCenter = 0.0;

    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera webcam;
    private Encoder.BlueObjectPositionPipeline blueObjectPipeline;
    private DcMotor encoderR;
    private DcMotor encoderC;// Assuming odometry pod is connected to this motor

    // Constants for encoder counts per inch and robot width
    private static final double COUNTS_PER_INCH = 2000; // Adjust this value based on your robot's setup
    private static final double ROBOT_WIDTH = 14.34; // Distance between wheels in inches

    @Override
    public void runOpMode() {


        lFront = hardwareMap.get(DcMotor.class, "lFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        rBack = hardwareMap.get(DcMotor.class, "rBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        bucket = hardwareMap.get(Servo.class, "bucket");
        bucketarm = hardwareMap.get(Servo.class, "bucketarm");
        bucketR = hardwareMap.get(Servo.class, "bucketR");
        bucketL = hardwareMap.get(Servo.class, "bucketL");

       // encoderR = hardwareMap.get(DcMotor.class, "encoderR");
        // encoderC = hardwareMap.get(DcMotor.class, "encoderC");
        // rightEncoder =

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lFront.setDirection(DcMotor.Direction.FORWARD);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.REVERSE);


       // encoderR.setDirection(DcMotor.Direction.REVERSE);
       // encoderC.setDirection(DcMotor.Direction.REVERSE);

        lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Specify the camera name and connection type
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Create the OpenCV pipeline
        blueObjectPipeline = new Encoder.BlueObjectPositionPipeline();

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

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Perform actions based on the object position
            switch (blueObjectPipeline.getObjectPosition()) {
                case CENTER:
                    // Move the robot left
                    moveForward(20,1);
                    strafe(5,0);
                    intake.setPower(-0.1);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                        telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                    moveForward(-10,0);
                    telemetry.addData("Status", "Center");    //
                    telemetry.update();
                    break;
                case RIGHT:
                    // Move the robot right
                    moveForward(20,0);
                    strafe(5,0);
                    intake.setPower(-0.1);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                        telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                    moveForward(-10,0);
                    telemetry.addData("Status", "Right");    //
                    telemetry.update();
                    break;
                case UNKNOWN:
                    // Do nothing or implement default behavior
                    moveForward(10,0);
                    strafe(5,0);
                    intake.setPower(-0.1);
                    runtime.reset();
                    while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                        telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                    moveForward(-10,0);
                    telemetry.addData("Status", "Left");    //
                    telemetry.update();
                    break;
            }

            telemetry.addData("Object Position", blueObjectPipeline.getObjectPosition());
            telemetry.update();
        }
        sleep(1000000);
        // Stop streaming and close the camera
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        // Move the robot 20 inches forward

    }

    public void moveForward(double dist, double dir) {

        double start = lBack.getCurrentPosition();
        encoderRight = start;
        telemetry.addData("Object Position", encoderRight);
        telemetry.update();
        sleep(5000);
        dist = dist * (192 + 15) * 4;  // 192 clicks per inch

        lFront.setPower( dir * FORWARD_SPEED );
        lBack.setPower( dir * FORWARD_SPEED );
        rFront.setPower(dir * FORWARD_SPEED);
        rBack.setPower(dir * FORWARD_SPEED);


        //runtime.reset();
        while (opModeIsActive() && (Math.abs(encoderRight - start)) < dist){
            encoderRight = lBack.getCurrentPosition();

        }

        // Step 4:  park
        lFront.setPower(0);
        rFront.setPower(0);
        lBack.setPower(0);
        rBack.setPower(0);

        telemetry.addData("Object Position", encoderRight);
        telemetry.update();
        sleep(5000);
        sleep (10);

        telemetry.update();

    }

    private void turn(double degrees) {
        // Calculate the distance each wheel needs to travel to turn the robot
        double inchesToTravel = Math.PI * ROBOT_WIDTH * degrees / 360.0;

        // Convert the distance to encoder counts
        int targetCounts = (int) (inchesToTravel * COUNTS_PER_INCH);

        // Reset encoders
        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set target position
        lFront.setTargetPosition(-targetCounts);
        lBack.setTargetPosition(-targetCounts);
        rFront.setTargetPosition(targetCounts);
        rBack.setTargetPosition(targetCounts);// Reverse one wheel to make the robot turn

        // Set motor power
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();

        // Keep looping while we are still active, and motors are still running
       // while (opModeIsActive() && lFront.isBusy() && lBack.isBusy() && rFront.isBusy() && rBack.isBusy()) {
            // Do nothing until the motors reach the target position
        }
    //}

    public void strafe(double strafeDist, int strafeDirection) {
        double strafeSpeed = 0.4;
        strafeDist = strafeDist * 261;

        encoderCenter = rFront.getCurrentPosition();
        double start = encoderCenter;

        if (strafeDirection > 0) {

            // strafe right

            lFront.setPower(-strafeSpeed -0.012);//correct value to go straight probably
            lBack.setPower(strafeSpeed + 0.0); //between 0.01 and 0.015
            rFront.setPower(strafeSpeed + 0.012);
            rBack.setPower(-strafeSpeed - 0.0);
        } else {

            // strafe left

            lFront.setPower(strafeSpeed + 0.012);
            lBack.setPower(-strafeSpeed);
            rFront.setPower(-strafeSpeed - 0.012);
            rBack.setPower(strafeSpeed);
        }


        while (opModeIsActive() && (Math.abs(encoderCenter - start)) < strafeDist){
            encoderCenter = rFront.getCurrentPosition();

        }


        // Step 4:  park
        lFront.setPower(0);
        rFront.setPower(0);
        lBack.setPower(0);
        rBack.setPower(0);
        sleep (500);

    }

    public static class BlueObjectPositionPipeline extends OpenCvPipeline {

        private Mat hsvImage;
        private Mat mask;
        private Mat hierarchy;

        private enum ObjectPosition {
            CENTER, RIGHT, UNKNOWN
        }

        private Encoder.BlueObjectPositionPipeline.ObjectPosition objectPosition = Encoder.BlueObjectPositionPipeline.ObjectPosition.UNKNOWN;

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
                    objectPosition = Encoder.BlueObjectPositionPipeline.ObjectPosition.CENTER;
                } else {
                    objectPosition = Encoder.BlueObjectPositionPipeline.ObjectPosition.RIGHT;
                }

                // Draw the contour with the largest area on the original image
                Imgproc.drawContours(input, contours, maxAreaIndex, new Scalar(0, 255, 0), 2);
            } else {
                objectPosition = Encoder.BlueObjectPositionPipeline.ObjectPosition.UNKNOWN;
            }

            return input;
        }

        public Encoder.BlueObjectPositionPipeline.ObjectPosition getObjectPosition() {
            return objectPosition;
        }
    }
}
