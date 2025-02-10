package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="2025TeleOp", group="TeleOp")
public class ArmTestTeleop extends OpMode {

    // Define motor and joystick
    private DcMotor armMotor;

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    private Servo wrist = null;

    private Servo joint = null;

    private CRServo R = null;
    private CRServo L = null;

    private DcMotor arm = null;
    private DcMotor lift = null;

    private ColorSensor colorSensor;


    public static final double STRAFEFAST = 1.0;    //  fastStrafe
    public double strafeSpeed = STRAFEFAST;

    // Encoder limits for the arm
    private static final int MAX_ARM_POSITION =  13240;  // Replace with your maximum encoder value
    private static final int MIN_ARM_POSITION = -111110; // Encoder value for the bottom position

    private static final double SERVO_MIN_POS = 0; // Minimum position for the servo
    private static final double SERVO_MAX_POS = 0.70; // Maximum position for the servo
private final  double INCREMENT = 0.01;
private  double servoPosition = 0.60;
    private  double pastPosition = 0;
    private static final double WRISTUP = 0.390;
    private static final double WRISTDOWN = 0.700;
    double wristValue = WRISTUP;
    double jointValue = 0.50;


    String detectedColor = "Unknown";

    @Override
    public void init() {
        // Initialize the arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        R = hardwareMap.get(CRServo.class, "R");
        L = hardwareMap.get(CRServo.class, "L");
        joint = hardwareMap.get(Servo.class, "joint");



        lFront = hardwareMap.get(DcMotor.class, "lFront");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rBack = hardwareMap.get(DcMotor.class, "rBack");
        lift = hardwareMap.get(DcMotor.class, "lift");

        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();


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

        // Get the current position of the arm using the encoder
        int currentPosition = armMotor.getCurrentPosition();

        // Get the y-axis value from the left joystick (range: -1.0 to 1.0)
        //int wristValue = (int) -gamepad2.left_stick_y;  // Invert for correct direction
        if ( gamepad2.left_stick_y >= 0.8 ){
            wristValue -= 0.002;
            if (wristValue >= WRISTDOWN) {
                wristValue = WRISTDOWN;
            }
            wrist.setPosition(wristValue);

        }
        if ( gamepad2.left_stick_y <= -0.8 ){
            wristValue += 0.002;
            if (wristValue <= WRISTUP) {
                wristValue = WRISTUP;
            }
            wrist.setPosition(wristValue);

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
                // outtake
                R.setPower(.3);
                L.setPower(-.3);
            } else if (gamepad2.left_trigger > 0.5){
                // intake
                R.setPower(-1);
                L.setPower(1);
            } else {
                // neutral
                R.setPower(0);
                L.setPower(0);
            }
            if (gamepad2.left_bumper) {
                jointValue -= 0.002;
                joint.setPosition(jointValue);
            }
        if (gamepad2.right_bumper) {
            jointValue += 0.002;
            joint.setPosition(jointValue);
        }
        if (gamepad2.y) {
            wrist.setPosition(.37);
        }
        if (gamepad2.x) {
            wrist.setPosition(.37);
        }
        if (gamepad2.b) {
            joint.setPosition(0.700);
        }
        if (gamepad2.a) {
            wrist.setPosition(0.700);
        }

            if (gamepad1.dpad_up) {
                lFront.setPower(0.5);
                lBack.setPower(0.5);
                rFront.setPower(0.5);
                rBack.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                lFront.setPower(-0.5);
                lBack.setPower(-0.5);
                rFront.setPower(-0.5);
                rBack.setPower(-0.5);
            } else if (gamepad1.dpad_left) {
                lFront.setPower(-0.5);
                lBack.setPower(0.5);
                rFront.setPower(0.5);
                rBack.setPower(-0.5);
            } else if (gamepad1.dpad_right) {
                lFront.setPower(0.5);
                lBack.setPower(-0.5);
                rFront.setPower(-0.5);
                rBack.setPower(0.5);

            }
        if (gamepad1.right_trigger > 0.1) {
            // move to open claw.
            lift.setPower(1);
            telemetry.update();
            //launcher.setPosition(LAUNCHER);
        } else if (gamepad1.left_trigger > 0.1) {
            // move to close claw.
            lift.setPower(-1);
            telemetry.update();
        } else {
            lift.setPower(0);
        }
        // Get the joystick Y value (assumed to be gamepad2 right stick Y)
        double joystickInput = -gamepad2.right_stick_y; // Negate if positive should be up

        // Move the arm if within limits
        if (joystickInput > 0.5) {
            // Moving up, but only if we haven't reached the maximum position
            armMotor.setPower(joystickInput);
        } else if (joystickInput < -0.5) {
            // Moving down, but only if we haven't reached the minimum position
            armMotor.setPower(joystickInput);
        } else {
            // Stop the arm if it's out of bounds or joystick is neutral
           armMotor.setPower(0.0001);
        }

        // Send telemetry to display arm position and joystick input
        telemetry.addData("Wrist Position", wristValue);
        telemetry.addData("Arm Position", currentPosition);
        telemetry.addData("joint Position", jointValue);
        telemetry.addData("Joystick Input", wristValue);
        telemetry.update();

      //  if (gamepad1.right_trigger)
    }
}