package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="test", group="TeleOp")
public class nief extends OpMode {

    // Define motor and joystick
    private DcMotor armMotor;

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    private CRServo wrist = null;

    private Servo claw = null;

    private DcMotor arm = null;
    private DcMotor lift = null;

    public static final double STRAFEFAST = 1.0;    //  fastStrafe
    public double strafeSpeed = STRAFEFAST;

    // Encoder limits for the arm
  //  private static final int MAX_ARM_POSITION = 3200;  // Replace with your maximum encoder value
  //  private static final int MIN_ARM_POSITION = 0; // Encoder value for the bottom position

    private static final double SERVO_MIN_POS = 0; // Minimum position for the servo
    private static final double SERVO_MAX_POS = 0.70; // Maximum position for the servo
    private final  double INCREMENT = 0.01;
    private  double servoPosition = 0.60;
    private  double pastPosition = 0;


    @Override
    public void init() {
        // Initialize the arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        lFront = hardwareMap.get(DcMotor.class, "lFront");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rBack = hardwareMap.get(DcMotor.class, "rBack");
        lift = hardwareMap.get(DcMotor.class, "lift");

        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);


        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Reset the motor's encoder and set to run using encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void loop() {

        double left;
        double right;

        // Get the y-axis value from the left joystick (range: -1.0 to 1.0)
        int wristValue = (int) gamepad2.left_stick_y;  // Invert for correct direction
        if ( wristValue == 1 && wristValue != 0 && wristValue != -1 ){
            wrist.setPower(-1);

        }
        if (wristValue == -1 && wristValue !=0 && wristValue != 1){
            wrist.setPower(0.90); //0.90
        }
        if(wristValue == 0 && wristValue != 1 && wristValue != -1){
            wrist.setPower(-0.14);
        }

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
        if (gamepad2.right_trigger > 0.5) {
            // move to open claw.
            claw.setPosition(1.3);
            telemetry.update();
            //launcher.setPosition(LAUNCHER);
        }
        if (gamepad2.left_trigger > 0.5) {
            // move to close claw.
            claw.setPosition(0.4);
            telemetry.update();
        }
        if (gamepad2.right_bumper) {
            claw.setPosition(0.7);
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
        if (gamepad1.right_trigger > 0.1) {
            // move to open claw.
            lift.setPower(0.8);
            telemetry.update();
            //launcher.setPosition(LAUNCHER);
        } else if (gamepad1.left_trigger > 0.1) {
            // move to close claw.
            lift.setPower(-0.4);
            telemetry.update();
        } else {
            lift.setPower(0);
        }
        // Get the joystick Y value (assumed to be gamepad2 right stick Y)
        double joystickInput = -gamepad2.right_stick_y; // Negate if positive should be up

        // Get the current position of the arm using the encoder
        int currentPosition = armMotor.getCurrentPosition();

        // Move the arm if within limits
        if (joystickInput > 0) {
            // Moving up, but only if we haven't reached the maximum position
            armMotor.setPower(joystickInput);
        }  else {
            // Stop the arm if it's out of bounds or joystick is neutral
            armMotor.setPower(0.001);
        }

        // Send telemetry to display arm position and joystick input
        telemetry.addData("Arm Position", currentPosition);
        telemetry.addData("Joystick Input", wristValue);
        telemetry.update();

        //  if (gamepad1.right_trigger)
    }
}