package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="FTCAUTORED2023", group="Robot")

public class FTCAUTORED2023 extends LinearOpMode {

    OpenCvCamera camera;

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    public DcMotor intake = null;

    public DcMotor extendo = null;
    public Servo bucket = null;

    public Servo bucketarm = null;

    private VideoCapture capture;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.6;

    public static final double BUCKET = 0;
    public static final double BUCKETRESET = 0.5;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

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

        lFront = hardwareMap.get(DcMotor.class, "lFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        rBack = hardwareMap.get(DcMotor.class, "rBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        bucket = hardwareMap.get(Servo.class, "bucket");
        bucketarm = hardwareMap.get(Servo.class, "bucketarm");




        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

            // Step 1:  Drive forward for 3 seconds
            lFront.setPower(FORWARD_SPEED);
            lBack.setPower(FORWARD_SPEED);
            rFront.setPower(FORWARD_SPEED);
            rBack.setPower(FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.39)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Drive backward for 3 seconds
            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);

                /*Front.setPower(-FORWARD_SPEED);
                lBack.setPower(FORWARD_SPEED);
                rFront.setPower(FORWARD_SPEED);
                rBack.setPower(-FORWARD_SPEED);*/
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            intake.setPower(-0.1);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.8)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            lFront.setPower(-FORWARD_SPEED);
            lBack.setPower(-FORWARD_SPEED);
            rFront.setPower(-FORWARD_SPEED);
            rBack.setPower(-FORWARD_SPEED);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.30)) {
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
            while (opModeIsActive() && (runtime.seconds() < 1.50)) {
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



            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();

                extendo.setPower(-1);

            }
            extendo.setPower(0);
            runtime.reset();

            while (opModeIsActive() && (runtime.seconds() < 1.3)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();

                bucket.setPosition(0);

            }

            while (opModeIsActive() && (runtime.seconds() < 1.1)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();

                extendo.setPower(1);
            }

            extendo.setPower(0);
            runtime.reset();

            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);
            intake.setPower(0);
            extendo.setPower(0);
            telemetry.addData("Path", "Complete");
            telemetry.update();
            runtime.reset();
            sleep(1000000);
        }
    }
}