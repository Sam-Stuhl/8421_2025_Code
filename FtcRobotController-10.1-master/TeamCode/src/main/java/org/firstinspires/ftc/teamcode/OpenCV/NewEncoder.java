package org.firstinspires.ftc.teamcode.OpenCV;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.videoio.VideoCapture;

@Autonomous(name = "FourWheelOdometryExample", group = "MyRobot")
public class NewEncoder extends LinearOpMode {
    private DcMotor lFront,rFront, lBack, rBack, intake, extendo;

    private Servo bucket,bucketarm,bucketR,bucketL;

    private VideoCapture capture;
    private DcMotor encoderR, encoderC, encoderL;
    private BNO055IMU imu;
    private Orientation lastAngles;

    // Constants for IMU
    private static final double P_TURN_COEFF = 0.1; // Proportional coefficient for turning

    // Constants (adjust these based on your robot)

    private static final double TURN_DIAMETER = 12.0; // Diameter of the circle the robot will turn on while making a 90-degree turn
    private static final double TURN_CIRCUMFERENCE = Math.PI * TURN_DIAMETER;
    private static final double TICKS_PER_REVOLUTION = 2000.0; // Number of encoder ticks per revolution
    private static final double DEAD_WHEEL_DIAMETER = 2.0; // Diameter of the dead wheels in inches
    private static final double WHEEL_BASE_WIDTH = 14.4; // Width between the two sets of wheels in inches
    private static final double ROBOT_WIDTH = 12.0; // Width between the two powered wheels in inches
    private static final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION * (TURN_CIRCUMFERENCE / WHEEL_BASE_WIDTH) / 360.0;


    private static final double TICKS_TO_INCHES = (Math.PI * DEAD_WHEEL_DIAMETER) / TICKS_PER_REVOLUTION; // Convert ticks to inches

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // IMU initialization
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        // Initialize hardware (configure motor names)
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

        encoderR = hardwareMap.get(DcMotor.class, "lBack");
        encoderL = hardwareMap.get(DcMotor.class, "lFront");
        encoderC = hardwareMap.get(DcMotor.class, "rFront");



        // Reverse the right motors if needed
        lFront.setDirection(DcMotor.Direction.FORWARD);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        extendo.setDirection(DcMotor.Direction.REVERSE);

        encoderR.setDirection(DcMotor.Direction.REVERSE);
        encoderC.setDirection(DcMotor.Direction.REVERSE);
        encoderL.setDirection(DcMotor.Direction.REVERSE);
        //rightEncoder.setD

       // encoderL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // encoderR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Example usage:
        moveForward(24); // Move forward 24 inches
      //  turn(90); // Turn 90 degrees
        // Other movement commands
    }

    private void moveForward(double distanceInches) {

        encoderL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = (int) ( distanceInches * TICKS_PER_REVOLUTION);
        int targetEncoderCounts = (int) (distanceInches / TICKS_TO_INCHES);

        double remainingDistance = distanceInches * TICKS_TO_INCHES * 2 -
                (lFront.getCurrentPosition() + rFront.getCurrentPosition()
                        + lBack.getCurrentPosition() + rBack.getCurrentPosition()) / 4;

        int mainWheelTargetPosition = (int) remainingDistance;

        lFront.setTargetPosition(mainWheelTargetPosition);
        rFront.setTargetPosition(mainWheelTargetPosition);
        lBack.setTargetPosition(mainWheelTargetPosition);
        rBack.setTargetPosition(mainWheelTargetPosition);

        lFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Set motor powers
        // Adjust motor powers accordingly to move forward
        lFront.setPower(0.5);
        rFront.setPower(0.5);
        lBack.setPower(0.5);
        rBack.setPower(0.5);

        encoderL.setTargetPosition(targetEncoderCounts);
        encoderR.setTargetPosition(targetEncoderCounts);
        encoderL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoderR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && (lFront.isBusy() || rFront.isBusy() || lBack.isBusy() || rBack.isBusy())) {
            telemetry.addData("Status", "Moving Forward");
            telemetry.addData("Left Front Position", lFront.getCurrentPosition());
            telemetry.addData("Right Front Position", rFront.getCurrentPosition());
            telemetry.addData("Left Rear Position", lBack.getCurrentPosition());
            telemetry.addData("Right Rear Position", rBack.getCurrentPosition());
            telemetry.update();
        }

        // Wait until movement is complete
      //  while (opModeIsActive() &&
       //         (encoderL.isBusy() || encoderR.isBusy())) {
            // Update odometry (if needed)
      //  }

 /*
        // Set motor powers and target positions
        leftFront.setTargetPosition(targetTicks);
        rightFront.setTargetPosition(targetTicks);
        leftRear.setTargetPosition(targetTicks);
        rightRear.setTargetPosition(targetTicks);

        // Set motor run mode to RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftRear.setPower(0.5);
        rightRear.setPower(0.5);

        // Wait until movement is complete
        while (opModeIsActive() &&
                leftFront.isBusy() && rightFront.isBusy() &&
                leftRear.isBusy() && rightRear.isBusy()) {
            // Update odometry (if needed)
        } */

        // Stop motors
        lFront.setPower(0);
        rFront.setPower(0);
        lBack.setPower(0);
        rBack.setPower(0);
        runtime.reset();
    }
/*
    private void turn(double angleDegrees) {
        int targetPosition = (int) (angleDegrees * TICKS_PER_DEGREE);
        double targetAngle = getAngle() + angleDegrees;
        double error;
        double steer;

        // Loop while absolute error is greater than tolerance
        do {
            error = getError(targetAngle);
            steer = error * P_TURN_COEFF;

            // Set motor powers
            lFront.setPower(-steer);
            rFront.setPower(steer);
            lBack.setPower(-steer);
            rBack.setPower(steer);
        } while (opModeIsActive() && Math.abs(error) > 1);

        // Stop motors
        lFront.setPower(0);
        rFront.setPower(0);
        lBack.setPower(0);
        rBack.setPower(0);
        runtime.reset();
    }

/*
        double distanceInches = Math.toRadians(angleDegrees) * WHEEL_BASE_WIDTH / 2.0;

        // Calculate target ticks for each wheel
        int targetTicks = (int) (distanceInches * TICKS_PER_DEGREE);

        // Set motor powers and target positions
        leftFront.setTargetPosition(-targetTicks);
        rightFront.setTargetPosition(targetTicks);
        leftRear.setTargetPosition(-targetTicks);
        rightRear.setTargetPosition(targetTicks);

        // Set motor run mode to RUN_TO_POSITION
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor powers
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftRear.setPower(0.5);
        rightRear.setPower(0.5);

        // Wait until movement is complete
        while (opModeIsActive() &&
                leftFront.isBusy() && rightFront.isBusy() &&
                leftRear.isBusy() && rightRear.isBusy()) {
            // Update odometry (if needed)
        }

        // Stop motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    */
 /*   private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    // Method to calculate error in turning
    private double getError(double targetAngle) {
        double robotError;
        robotError = targetAngle - getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    } */
}