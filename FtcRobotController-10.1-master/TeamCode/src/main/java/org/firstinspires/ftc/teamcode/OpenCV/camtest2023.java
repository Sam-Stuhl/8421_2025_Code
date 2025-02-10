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

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BlueObjectAutonomous", group = "Autonomous")
public class camtest2023 extends LinearOpMode {

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
    double encoderLeft = 0.0;

    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera webcam;
    private BlueObjectPositionPipeline blueObjectPipeline;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

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
        // rightEncoder =

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.FORWARD);
        //rightEncoder.setD

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
                    moveCenter();
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
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        intake.setPower(-0.1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        rFront.setPower(-FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive forward for 3 seconds
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.9)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        rFront.setPower(-FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 4:  Drive Backward for 3 Seconds
        extendo.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        extendo.setPower(0);

        bucketarm.setPosition(0.25);
        bucket.setPosition(0.67); //was .7
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        bucketR.setPosition(1);
        bucketL.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        bucketR.setPosition(0);
        bucketL.setPosition(1);
        bucket.setPosition(0.40);
        bucketarm.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        extendo.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 5:  Stop
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        extendo.setPower(0);
        intake.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        extendo.setPower(0);
        intake.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100000);
    }

    private void moveRight() {
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        rFront.setPower(-FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        intake.setPower(-0.1);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        rFront.setPower(-FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 4:  Drive Backward for 3 Seconds
        extendo.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        extendo.setPower(0);

        bucketarm.setPosition(0.25);
        bucket.setPosition(0.67); //was .7
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        bucketR.setPosition(1);
        bucketL.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        bucketR.setPosition(0);
        bucketL.setPosition(1);
        bucket.setPosition(0.40);
        bucketarm.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        extendo.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 5:  Stop
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        extendo.setPower(0);
        intake.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        extendo.setPower(0);
        intake.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100000);


    }

    private void stopRobot() {
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        intake.setPower(-0.2);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        rFront.setPower(-FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.2)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(-FORWARD_SPEED);
        rFront.setPower(-FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive backward for 3 seconds
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        // Step 4:  Drive Backward for 3 Seconds
        extendo.setPower(-FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.8)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        extendo.setPower(0);

        bucketarm.setPosition(0.25);
        bucket.setPosition(0.67); //was .7
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        bucketR.setPosition(1);
        bucketL.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        bucketR.setPosition(0);
        bucketL.setPosition(1);
        bucket.setPosition(0.40);
        bucketarm.setPosition(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        extendo.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 5:  Stop
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        extendo.setPower(0);
        intake.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
        lFront.setPower(-FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(FORWARD_SPEED);
        lBack.setPower(FORWARD_SPEED);
        rFront.setPower(FORWARD_SPEED);
        rBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);
        extendo.setPower(0);
        intake.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100000);
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