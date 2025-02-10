
package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This particular OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 * <p>
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "FourWD", group = "Robot")
public class FourWD extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;






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



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);
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

            left = -(gamepad1.left_stick_y);
            right = (gamepad1.right_stick_y);


            lFront.setPower(left);
            lBack.setPower(left);
            rFront.setPower(-right);
            rBack.setPower(-right);

            if ((-gamepad1.left_stick_x > 0.5 && -gamepad1.right_stick_x > 0.5)) {
                lFront.setPower(strafeSpeed);
                lBack.setPower(-strafeSpeed);
                rFront.setPower(strafeSpeed);
                rBack.setPower(-strafeSpeed);
            }
            if ((-gamepad1.left_stick_x < -0.5 && -gamepad1.right_stick_x < -0.5)) {
                lFront.setPower(-strafeSpeed);
                lBack.setPower(strafeSpeed);
                rFront.setPower(-strafeSpeed);
                rBack.setPower(strafeSpeed);
            }



            if (gamepad1.dpad_up) {
                lFront.setPower(0.5);
                lBack.setPower(-0.5);
                rFront.setPower(0.5);
                rBack.setPower(-0.5);
            } else if (gamepad1.dpad_down) {
                lFront.setPower(-0.5);
                lBack.setPower(-0.5);
                rFront.setPower(0.5);
                rBack.setPower(0.5);
            } else if (gamepad1.dpad_left) {
                lFront.setPower(0.5);
                lBack.setPower(-0.5);
                rFront.setPower(0.5);
                rBack.setPower(-0.5);
            } else if (gamepad1.dpad_right) {
                lFront.setPower(-0.5);
                lBack.setPower(0.5);
                rFront.setPower(-0.5);
                rBack.setPower(0.5);
            }
        }
    }
}
