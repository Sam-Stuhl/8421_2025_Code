
package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import java.util.Locale;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 * <p>
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "FTCTeleOP2023", group = "Robot")
public class FTCTeleOP2023 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    private Servo launcher = null;

    private Servo bucketR = null;

    private Servo bucketL = null;

    private IMU imu = null;


    public DcMotor intake = null;

    public DcMotor lift = null;

    public DcMotor extendo = null;
    public DcMotor extendo2 = null;

    public Servo bucket = null;

    public Servo bucketarm = null;
    
    public static final double STRAFEFAST = 1.0;    //  fastStrafe
    public double strafeSpeed = STRAFEFAST;



    // CLAW
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double LAUNCHER = 0;
    public static final double LAUNCHERESET = 1;
    // sets rate to move servo


    // TIME

    private ElapsedTime runtime = new ElapsedTime();

    private int targetPosition = 0;

    @Override
    public void runOpMode() {

        // Define and Initialize Motors
        lFront = hardwareMap.get(DcMotor.class, "lFront");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rBack = hardwareMap.get(DcMotor.class, "rBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotor.class, "lift");
        launcher = hardwareMap.get(Servo.class, "launcher");
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo2 = hardwareMap.get(DcMotor.class, "extendo2");
        bucket = hardwareMap.get(Servo.class, "bucket");
        bucketarm = hardwareMap.get(Servo.class, "bucketarm");
        bucketR = hardwareMap.get(Servo.class, "bucketR");
        bucketL = hardwareMap.get(Servo.class, "bucketL");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendo2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launcher.setDirection(Servo.Direction.);

        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // extendo2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double Joystick2L_Input = gamepad1.left_stick_y;
        double left;
        double right;
        int one_touch_lift_up = 0;  // move lift up
        int one_touch_lift_down = 0;   //move lift down
        int yButton = 0;    //reflects whether the user pressed "Y"
        int xButton = 0;     //reflects whether the user pressed "Y"

        // int low_goal = 0;    // flag to move lift to low goal position.
        // double ground = 0;
        // double strafe = 0;
        // double liftCurrent = 0.0;
        // double targetPosition = 0.0;
        double speed = 1.5;

        // ground = lift.getCurrentPosition();


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        strafeSpeed = STRAFEFAST;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)

            // SLOW MODE
            if (gamepad1.left_bumper) {
                strafeSpeed = STRAFEFAST;
                speed = 1;
            }
            if (gamepad1.right_bumper) {
                strafeSpeed = STRAFEFAST;
                speed = 1.5;
            }

            //strafe = gamepad1.right_stick_x;

            //if ((gamepad1.left_trigger > 0.5) || (gamepad1.right_trigger > 0.5)) {

          /*  if (gamepad1.left_trigger > 0.5) {
                lFront.setPower(strafeSpeed);
                lBack.setPower(-strafeSpeed);
                rFront.setPower(-strafeSpeed);
                rBack.setPower(strafeSpeed);
            }
            if (gamepad1.right_trigger > 0.5) {
                lFront.setPower(-strafeSpeed);
                lBack.setPower(strafeSpeed);
                rFront.setPower(strafeSpeed);
                rBack.setPower(-strafeSpeed);
            } */

            left = 0;
            right = 0;
            //     *****************  End strafe code *****************************************
           /* left = -(gamepad1.left_stick_y / speed);
            right = -(gamepad1.right_stick_y / speed);
            if (((left > 0.7) || (left < -0.7)) && ((right > 0.7) || (right < -0.7))) {
                left = (left / 1.5);    // REEEEEEEEEE MOOOODDDDEEEEE
                right = (right / 1.5);

           } */
            /*else {
                left = (left / 2);      // slow mode
                right = (right / 2);
            }
            */

            left = -(gamepad1.left_stick_y);
            right = (gamepad1.right_stick_y);

            lFront.setPower(left);
            lBack.setPower(left);
            rFront.setPower(-right);
            rBack.setPower(-right);

            if ((-gamepad1.left_stick_x > 0.5 && -gamepad1.right_stick_x > 0.5)) {
                lFront.setPower(-strafeSpeed);
                lBack.setPower(strafeSpeed);
                rFront.setPower(strafeSpeed);
                rBack.setPower(-strafeSpeed);
            }
            if ((-gamepad1.left_stick_x < -0.5 && -gamepad1.right_stick_x < -0.5)) {
                lFront.setPower(strafeSpeed);
                lBack.setPower(-strafeSpeed);
                rFront.setPower(-strafeSpeed);
                rBack.setPower(strafeSpeed);
            }

            if (gamepad1.right_trigger > 0.36) {
                // move to open claw.
                intake.setPower(0.87);
                telemetry.update();
                //launcher.setPosition(LAUNCHER);
            } else if (gamepad1.left_trigger > 0.4) {
                // move to close claw.
                intake.setPower(-0.4);
                telemetry.update();
            } else {
                intake.setPower(0);
            }
            if (gamepad1.dpad_up) {
                lFront.setPower(-0.5);
                lBack.setPower(0.5);
                rFront.setPower(0.5);
                rBack.setPower(-0.5);
            } else if (gamepad1.dpad_down) {
                lFront.setPower(0.5);
                lBack.setPower(-0.5);
                rFront.setPower(-0.5);
                rBack.setPower(0.5);
            } else if (gamepad1.dpad_left) {
                lFront.setPower(0.5);
                lBack.setPower(0.5);
                rFront.setPower(0.5);
                rBack.setPower(0.5);
            } else if (gamepad1.dpad_right) {
                lFront.setPower(-0.5);
                lBack.setPower(-0.5);
                rFront.setPower(-0.5);
                rBack.setPower(-0.5);
            }


          /*  if ((-gamepad1.left_stick_y < 0.3 && -gamepad1.left_stick_y > -0.3)

                    //||
                    &&
                    (-gamepad1.right_stick_y < 0.3 && -gamepad1.right_stick_y > -0.3)) {

                lBack.setPower(0);
                rFront.setPower(0);
                rBack.setPower(0);
                lFront.setPower(0);

            }*/


            //else {


                    /*left = (left / 2);      // slow mode
                    right = (right / 2);*/
            // }

            /************************* GAMEPAD 2 **************************/
            // Use gamepad left & right Triggers to use intake
            //A    if ((gamepad2.left_trigger > 0.5) || (gamepad2.right_trigger > 0.5)) {
            /**************INTAKE***************
             if (gamepad2.right_trigger > 0.5) {
             // move to open claw.
             intake.setPower(1.0);
             telemetry.update();
             //launcher.setPosition(LAUNCHER);
             } else if (gamepad2.left_trigger > 0.3) {
             // move to close claw.
             intake.setPower(-0.8);
             telemetry.update();
             } else {
             intake.setPower(0);
             } */
            /**************LAUNCHER****************/
            if (gamepad2.b) {
                launcher.setPosition(LAUNCHER);
            }
            if (gamepad2.x) {
                launcher.setPosition(LAUNCHERESET);
            }
            /**************DPAD****************/
            if (gamepad2.dpad_up) {
                bucketarm.setPosition(0.10);
                bucket.setPosition(0.30); //was .7
            }
            if (gamepad2.dpad_down) {
                bucket.setPosition(0.12);
                bucketarm.setPosition(0.65);
            }
            if (gamepad2.dpad_left) {
                bucket.setPosition(0.20);

            }
            if (gamepad2.dpad_right) {
                bucket.setPosition(0.12);
            }
            /****************Servo_BUCKET*******************/
            if (gamepad2.y) {
                bucketR.setPosition(1);
                bucketL.setPosition(0);
            }
            if (gamepad2.right_trigger > 0.1) {
                bucketR.setPosition(0.70);
            }
            if (gamepad2.left_trigger > 0.1) {
                bucketL.setPosition(0.25);

            }
            /**************EXTENDO****************/
            if (gamepad2.right_stick_y < 0.1) {
                extendo.setPower(-0.03);
              //  extendo2.setPower(0.03);
            }

            /**************EXTENDO&LIFT****************/
            lift.setPower(gamepad2.left_stick_y);


            extendo.setPower(-gamepad2.right_stick_y);
            extendo2.setPower(gamepad2.right_stick_y);


        }

         /*   left = -(gamepad2.left_stick_y / speed);
            right = -(gamepad2.right_stick_y / speed);
            if (((left > 0.) || (left < -0.7)) && ((right > 0.7) || (right < -0.7))) {
                left = (left / 1.0);
                right = (right / 1.0);
            }
            else {
                left = (left / 2);
                right = (right / 2);
            }
            lift.setPower(left);
            lift.setPower(right);/*

          */

        // else {

        // }
        //
        //   this resets the ground  because the lift rotation counter keeps drifting.
        //
           /* if (gamepad2.right_bumper) {
                ground = lift.getCurrentPosition();*/
        //}

            /*if (gamepad2.a) {
                lowGoal = true;
                mediumGoal = false;
                highGoal = false;
                targetPosition = LOWGOAL;
            }
            if (gamepad2.y) {
                mediumGoal = false;
                lowGoal = false;
                highGoal = true;
                targetPosition = HIGHGOAL;
            }
            if (gamepad2.x) {
                mediumGoal = true;
                lowGoal = false;
                highGoal = false;
                lift.setPower(0);

             */
        // }
           /* if (!mediumGoal && !lowGoal && !highGoal) {
                if (gamepad2.right_stick_y < -0.7) {
                    lift.setPower(0.99);  // lift up
                } else if (gamepad2.right_stick_y > 0.7) {
                    lift.setPower(-0.99);
                } else {
                    lift.setPower(0.07); */


        //   Move lift down 6 inches

          /*  if (gamepad2.b) {
                lift.setPower(-0.7);
                runtime.reset();
                while ((runtime.seconds() < .04)) {
                    telemetry.addData("lift moving down", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                }
                lift.setPower(0);
                targetPosition = lift.getCurrentPosition();
                lowGoal = false;
                mediumGoal = false;
                highGoal = false;

            }
            // medium goal
            if (mediumGoal) {

                if (lift.getCurrentPosition() < (Math.abs(ground + MEDIUMGOAL))) {
                    //  lift too low, go up
                    lift.setPower(0.95);

                } else if (lift.getCurrentPosition() > (Math.abs(ground + MEDIUMGOAL + 100))) {
                    // lift too high
                    lift.setPower(-0.4);

                } else {
                    // lift in sweety spot
                    mediumGoal = false;
                    lowGoal = false;
                    mediumGoal = false;
                    highGoal = false;
                    lift.setPower(0.05);

                }
                targetPosition = lift.getCurrentPosition();
            }
            // high goal
            if (highGoal) {

                if (lift.getCurrentPosition() < (Math.abs(ground + HIGHGOAL))) {
                    //  lift too low, go up
                    lift.setPower(0.95);
                    telemetry.addData("lift moving up", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else if (lift.getCurrentPosition() > (Math.abs(ground + HIGHGOAL + 100))) {
                    // lift too high
                    lift.setPower(-0.4);
                    telemetry.addData("lift moving DOWN", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else {
                    // lift in sweet spot
                    highGoal = false;
                    lowGoal = false;
                    mediumGoal = false;
                    highGoal = false;
                    lift.setPower(0.05);

                }
                targetPosition = lift.getCurrentPosition();
            }
            // low goal
            if (lowGoal) {

                if (lift.getCurrentPosition() < (Math.abs(ground + LOWGOAL))) {
                    //  lift too low, go up
                    lift.setPower(0.95);
                    telemetry.addData("lift moving up", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else if (lift.getCurrentPosition() > (Math.abs(ground + LOWGOAL + 100))) {
                    // lift too high
                    lift.setPower(-0.4);
                    telemetry.addData("lift moving up", "Leg 1: %4d S Elapsed", lift.getCurrentPosition());
                    telemetry.update();
                } else {
                    // lift in sweety spot
                    lowGoal = false;
                    lowGoal = false;
                    mediumGoal = false;
                    highGoal = false;
                    lift.setPower(0.05);

                }
                targetPosition = lift.getCurrentPosition();
            }


            // Pace this loop so jaw action is reasonable speed.
            sleep(1);

           */
    }
}
