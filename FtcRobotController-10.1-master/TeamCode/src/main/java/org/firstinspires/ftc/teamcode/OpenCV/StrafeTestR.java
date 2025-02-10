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

@Autonomous(name="StrafeTestR", group="Robot")

public class StrafeTestR extends LinearOpMode {

    OpenCvCamera camera;

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    public DcMotor intake = null;

    public DcMotor lift = null;
    public Servo launcher = null;

    private VideoCapture capture;

    public static final double LAUNCHER = 1;

    static final double FORWARD_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double BACKWARD_SPEED = -0.6;

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
        lift = hardwareMap.get(DcMotor.class, "lift");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        lift.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way


            // Step 4:  Drive Backward for 3 Seconds
           /* lift.setPower(FORWARD_SPEED);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.2)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }*/

            rFront.setPower(FORWARD_SPEED);
            lBack.setPower(-FORWARD_SPEED);
            lFront.setPower(-FORWARD_SPEED);
            rBack.setPower(-FORWARD_SPEED);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            // Step 5:  Stop
            lFront.setPower(0);
            lBack.setPower(0);
            rFront.setPower(0);
            rBack.setPower(0);
            lift.setPower(0);
            intake.setPower(0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(100000);
        }

    }
}
