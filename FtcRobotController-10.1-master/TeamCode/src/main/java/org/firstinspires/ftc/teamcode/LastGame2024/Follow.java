package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;



@Autonomous(name = "Blue Rectangle Tracking", group = "Autonomous")

public class Follow extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private static final Scalar LOWER_BLUE = new Scalar(100, 150, 0);
    private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);
    private static final double TURN_SPEED = 0.5; // Speed for turning
    private static final double MOVE_SPEED = 0.5; // Speed for moving forward
    private static final double STOP_SPEED = 0.0; // Stop speed
    private static final double AREA_THRESHOLD = 1000; // Area threshold for detection
    private static final int FRAME_WIDTH = 640; // Frame width
    private static final int FRAME_HEIGHT = 480; // Frame height

private OpenCvCamera camera;
private VideoCapture capture;
   // private OpenCvCamera openCvCamera;

    @Override
    public void runOpMode() {
        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        // Initialize motors
       // capture = new VideoCapture(0); // Adjust camera index as needed

        leftMotor = hardwareMap.get(DcMotor.class, "rFront"); // Replace with your motor names
        rightMotor = hardwareMap.get(DcMotor.class, "lFront");

        // Initialize the camera
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
            // Capture frame
            Mat frame = new Mat();
            capture.read(frame);

            // Convert to HSV
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

            // Create mask for blue color
            Mat mask = new Mat();
            Core.inRange(frame, LOWER_BLUE, UPPER_BLUE, mask);

            // Find contours of the blue rectangles
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // Process contours
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > AREA_THRESHOLD) { // Only consider large rectangles
                    Rect rect = Imgproc.boundingRect(contour);
                    // Draw rectangle on the frame (for debugging purposes)
                    Imgproc.rectangle(frame, rect.tl(), rect.br(), new Scalar(255, 0, 0), 2);

                    // Determine the center of the rectangle
                    Point center = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);

                    // Move towards the rectangle
                    moveTowardsRectangle(center);
                    break; // Exit after processing the first detected rectangle
                }
            }

            // Optional: Display the frame for debugging
            telemetry.addData("Contours Found: ", contours.size());
            telemetry.update();

            // Release the frame and mask
            frame.release();
            mask.release();
        }

        // Stop motors after the loop ends
        stopMotors();
    }

    private void moveTowardsRectangle(Point center) {
        // Calculate the center of the frame
        double frameCenterX = FRAME_WIDTH / 2;
        double frameCenterY = FRAME_HEIGHT / 2;

        // Determine offsets from center
        double xOffset = center.x - frameCenterX;
        double yOffset = center.y - frameCenterY;

        // Control robot movement
        if (Math.abs(xOffset) < 50 && Math.abs(yOffset) < 50) {
            // If the rectangle is close to the center, move forward
            setMotorPowers(MOVE_SPEED, MOVE_SPEED);
        } else if (xOffset < -50) {
            // If the rectangle is to the left, turn right
            setMotorPowers(TURN_SPEED, -TURN_SPEED);
        } else if (xOffset > 50) {
            // If the rectangle is to the right, turn left
            setMotorPowers(-TURN_SPEED, TURN_SPEED);
        } else if (yOffset < -50) {
            // If the rectangle is above, move slightly up (adjust this as needed)
            setMotorPowers(MOVE_SPEED, MOVE_SPEED);
        } else if (yOffset > 50) {
            // If the rectangle is below, move slightly down (adjust this as needed)
            setMotorPowers(-MOVE_SPEED, -MOVE_SPEED);
        } else {
            // Stop if not moving
            setMotorPowers(STOP_SPEED, STOP_SPEED);
        }
    }

    private void setMotorPowers(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    private void stopMotors() {
        leftMotor.setPower(STOP_SPEED);
        rightMotor.setPower(STOP_SPEED);
    }
}

