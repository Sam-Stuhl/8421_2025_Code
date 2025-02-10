package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name = "Odometry Autonomous", group = "Linear Opmode")
public class newAuto extends LinearOpMode {

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
    private static final int MAX_ARM_POSITION = 3200;  // Replace with your maximum encoder value
    private static final int MIN_ARM_POSITION = 0; // Encoder value for the bottom position

    private static final double SERVO_MIN_POS = 0; // Minimum position for the servo
    private static final double SERVO_MAX_POS = 0.70; // Maximum position for the servo
    private final  double INCREMENT = 0.01;
    private  double servoPosition = 0.60;
    private  double pastPosition = 0;


    private ElapsedTime runtime = new ElapsedTime();

    // Declare hardware variables
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor strafeMotor = null; // For horizontal (left-right) movement

    private DcMotor verticalLeftOdometry = null;
    private DcMotor verticalRightOdometry = null;
    private DcMotor horizontalOdometry = null;

    // Robot position tracking
    private double X_position = 0;
    private double Y_position = 0;
    private double theta = 0; // Robot's current orientation (heading)

    @Override
    public void runOpMode() {
        // Initialize hardware

        armMotor = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(CRServo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        lFront = hardwareMap.get(DcMotor.class, "lFront");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rBack = hardwareMap.get(DcMotor.class, "rBack");
        lift = hardwareMap.get(DcMotor.class, "lift");

        verticalLeftOdometry = hardwareMap.get(DcMotor.class, "lFront");
        verticalRightOdometry = hardwareMap.get(DcMotor.class, "lBack");
        horizontalOdometry = hardwareMap.get(DcMotor.class, "rBack");

        // Set motor directions (assuming the motors face opposite directions)
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

       // leftMotor.setDirection(DcMotor.Direction.REVERSE);
       // rightMotor.setDirection(DcMotor.Direction.REVERSE);
       // strafeMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset odometry pod encoders
        verticalLeftOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRightOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoder feedback
        verticalLeftOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRightOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();

        // Autonomous routine
        if (opModeIsActive()) {
            moveForward(24);   // Move forward 24 inches
            moveRight(24);     // Move right 24 inches
            moveBackward(24);  // Move backward 24 inches
            moveLeft(24);      // Move left 24 inches
        }
    }

    // Calculate robot position based on odometry encoder values
    public void updatePosition() {
        double deltaY = (verticalLeftOdometry.getCurrentPosition() + verticalRightOdometry.getCurrentPosition()) / 2.0;
        double deltaX = horizontalOdometry.getCurrentPosition();

        X_position += deltaX * Math.cos(theta) - deltaY * Math.sin(theta);
        Y_position += deltaX * Math.sin(theta) + deltaY * Math.cos(theta);

        telemetry.addData("X Position", X_position);
        telemetry.addData("Y Position", Y_position);
        telemetry.update();
    }

    // Function to move forward a specified distance in inches
    public void moveForward(double targetDistance) {
        double initialY = Y_position;

        while (opModeIsActive() && Math.abs(Y_position - initialY) < targetDistance) {
            updatePosition();
            setMotorPower(0.5, 0.5); // Move both motors forward
        }
        stopMotors();
    }

    // Function to move backward a specified distance in inches
    public void moveBackward(double targetDistance) {
        double initialY = Y_position;

        while (opModeIsActive() && Math.abs(Y_position - initialY) < targetDistance) {
            updatePosition();
            setMotorPower(-0.5, -0.5); // Move both motors backward
        }
        stopMotors();
    }

    // Function to move left (strafe) a specified distance in inches
    public void moveLeft(double targetDistance) {
        double initialX = X_position;

        while (opModeIsActive() && Math.abs(X_position - initialX) < targetDistance) {
            updatePosition();
            lFront.setPower(0.5);
            lBack.setPower(-0.5);
            rFront.setPower(-0.5);
            rBack.setPower(0.5);
        }
        stopMotors();
    }

    // Function to move right (strafe) a specified distance in inches
    public void moveRight(double targetDistance) {
        double initialX = X_position;

        while (opModeIsActive() && Math.abs(X_position - initialX) < targetDistance) {
            updatePosition();
            lFront.setPower(-0.5);
            lBack.setPower(0.5);
            rFront.setPower(0.5);
            rBack.setPower(-0.5); // Strafe right
        }
        stopMotors();
    }

    // Utility function to stop all motors
    public void stopMotors() {
        lFront.setPower(0);
        rFront.setPower(0);
        lBack.setPower(0);
    }

    // Set motor power for both sides of the drivetrain
    public void setMotorPower(double leftPower, double rightPower) {
        lFront.setPower(Range.clip(leftPower, -1.0, 1.0));
        rFront.setPower(Range.clip(rightPower, -1.0, 1.0));
       // lBack.setPower(Range.clip(leftPower, -1.0, 1.0));
       // rBack.setPower(Range.clip(rightPower, -1.0, 1.0));
    }
}
