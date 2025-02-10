package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="ColorAuto", group="Robot")

public class ColorAuto extends LinearOpMode {

    OpenCvCamera camera;

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;
    private VideoCapture capture;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.6;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeCamera();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Capture frame from camera
            Mat frame = new Mat();
            if (capture.read(frame)) {
                // Convert frame to HSV color space for better color detection
                Mat hsvFrame = new Mat();
                Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

                // Threshold for red color
                Mat redMask = new Mat();
                Core.inRange(hsvFrame, new Scalar(0, 100, 100), new Scalar(10, 255, 255), redMask);

                // Threshold for yellow color
                Mat yellowMask = new Mat();
                Core.inRange(hsvFrame, new Scalar(20, 100, 100), new Scalar(30, 255, 255), yellowMask);

                // Find contours of red objects
                Mat redContours = redMask.clone();
                List<MatOfPoint> redContoursList = new ArrayList<>();
                Imgproc.findContours(redContours, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Find contours of yellow objects
                Mat yellowContours = yellowMask.clone();
                List<MatOfPoint> yellowContoursList = new ArrayList<>();
                Imgproc.findContours(yellowContours, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                // Process red contours
                for (MatOfPoint contour : redContoursList) {
                    // Get center point of the contour
                    Moments moments = Imgproc.moments(contour);
                    double centerX = moments.get_m10() / moments.get_m00();
                    double centerY = moments.get_m01() / moments.get_m00();

                    // Implement robot movement logic based on red detection
                    // Example: Move forward if red object is detected
                    moveForward();
                }

                // Process yellow contours
                for (MatOfPoint contour : yellowContoursList) {
                    // Get center point of the contour
                    Moments moments = Imgproc.moments(contour);
                    double centerX = moments.get_m10() / moments.get_m00();
                    double centerY = moments.get_m01() / moments.get_m00();

                    // Implement robot movement logic based on yellow detection
                    // Example: Move backward if yellow object is detected
                    moveBackward();
                }
                lFront.setPower(0);
                lBack.setPower(0);
                rFront.setPower(0);
                rBack.setPower(0);

                sleep(1000);
            }
        }

    }

    private void initializeHardware() {
        // Initialize your hardware here (e.g., motors)
        lFront = hardwareMap.get(DcMotor.class, "lFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        rBack = hardwareMap.get(DcMotor.class, "rBack");


        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);
        // Set motor directions and other configurations as needed
    }

    private void initializeCamera() {
        // Initialize your camera (e.g., Webcam or Phone Camera)
        capture = new VideoCapture(0); // Adjust camera index as needed
        capture.open(0); // Open camera
    }

    private void moveForward() {
        // Implement robot movement logic to move forward
        telemetry.addLine("Cone");
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        }
    }

    private void moveBackward() {
        // Implement robot movement logic to move backward
        telemetry.addLine("Pole");
        lFront.setPower(BACKWARD_SPEED);
        lBack.setPower(BACKWARD_SPEED);
        rFront.setPower(BACKWARD_SPEED);
        rBack.setPower(BACKWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        }
    }
}