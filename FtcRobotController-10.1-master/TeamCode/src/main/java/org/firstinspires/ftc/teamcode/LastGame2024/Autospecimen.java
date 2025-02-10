package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autospecimen", group = "Linear Opmode")
public class Autospecimen extends LinearOpMode {

    private Servo wrist = null;

    private Servo claw = null;

    private DcMotor arm = null;

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;


    // Odometry wheel motors
    DcMotor leftEncoder, rightEncoder, horizontalEncoder;

    // Robot parameters
    final double COUNTS_PER_REVOLUTION_ARM = 3200;  // Example encoder counts per revolution for the arm motor
    final double INCHES_PER_REVOLUTION_ARM = 1.0;  // How many inches the arm lifts per motor revolution
    final double COUNTS_PER_INCH_ARM = COUNTS_PER_REVOLUTION_ARM / INCHES_PER_REVOLUTION_ARM;

    final double COUNTS_PER_INCH = 353.68;  // Based on wheel diameter and encoder resolution 353.68 2000
    final double ROBOT_DIAMETER = 15.0;  // Example robot diameter in inches, adjust for your robot

    // Position variables
    private double xPos = 0, yPos = 0;  // X, Y coordinates in inches
    private double heading = 0;  // Heading in degrees (handled via the two vertical encoders)
    private static final double WRISTUP = 0.13;
    private static final double WRISTDOWN = 0.95;
    double wristValue = WRISTUP;

    // Timer for loop
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware map

        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        leftEncoder = hardwareMap.get(DcMotor.class, "lFront");
        rightEncoder = hardwareMap.get(DcMotor.class, "lBack");
        horizontalEncoder = hardwareMap.get(DcMotor.class, "rBack");

        // Reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        wrist.setPosition(WRISTUP);
        closeClaw();

        // Wait for the start command
        waitForStart();

        // Start the update odometry thread
        Thread positionUpdate = new Thread(new OdometryGlobalPositionUpdate(leftEncoder, rightEncoder, horizontalEncoder, COUNTS_PER_INCH));
        positionUpdate.start();

        // Autonomous movement example
        moveToPosition(0, 12, 0.5);
        liftArmToHeight(0.5,1, 2);
        rotateWrist(0.75, 4);
        openClaw();
        rotateWrist(0.20, 0.5);
        liftArmToHeight(0,1, 2);
        moveToPosition(6, 0, 0.5);
        positionUpdate.interrupt();
        positionUpdate.start();
        moveToPosition(10, 5, 0.2);
        moveToPosition(6, 0, 0.5);

        rotateWrist(0.93, 1);
        rotateWrist(0.13, 10);
        singleSideTurn2(130, 0.5, true);
        positionUpdate.interrupt();
        positionUpdate.start();
        //turnInPlace(90, 0.5);
        //turnToHeading(80, 0.4);  // Turn to face 90 degrees (right turn)
        //moveToPosition(0, 6, 0.5);  // Move left by 12 inches

        // Stop the position update thread when done
        positionUpdate.interrupt();
    }

    // Method to move the robot to a given X, Y position
    public void moveToPosition(double targetXInches, double targetYInches, double power) {
        double distanceToX = targetXInches - xPos;
        double distanceToY = targetYInches - yPos;

        double minPower = 0.1;  // Minimum power to prevent the robot from stalling
        double maxPower = 0.5;  // Maximum power when far from the target

        while (opModeIsActive() && (Math.abs(distanceToX) > 2 || Math.abs(distanceToY) > 2)) {
            distanceToX = targetXInches - xPos;
            distanceToY = targetYInches - yPos;

            double distance = Math.hypot(distanceToX, distanceToY);
            double scaledPower = Math.max(minPower, Math.min(maxPower, distance / 10));  // Scale power based on distance

            double xPower = (distanceToX / distance) * scaledPower;
            double yPower = (distanceToY / distance) * scaledPower;


            //setMotorPowers(-yPower + xPower, -yPower - xPower, -yPower - xPower, -yPower + xPower);
            setMotorPowers(yPower + xPower, yPower - xPower, yPower - xPower, yPower + xPower);



            telemetry.addData("Target X", targetXInches);
            telemetry.addData("Target Y", targetYInches);
            telemetry.addData("Current X", xPos);
            telemetry.addData("Current Y", yPos);
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
        setMotorPowers(0, 0, 0, 0);  // Stop the robot once the target is reached
    }

    public void singleSideTurn2(double targetAngle, double power, boolean isLeftSideFixed) {
        // Calculate target encoder counts for the turn
        double turnDistanceInInches = (targetAngle / 360.0) * (Math.PI * ROBOT_DIAMETER); // Convert angle to arc length
        double targetCounts = turnDistanceInInches * COUNTS_PER_INCH;

        // Reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to RUN_WITHOUT_ENCODER to use odometry
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Determine which side is fixed and move the other side
        if (isLeftSideFixed) {
            // Right side moves; left side is stationary
            while (opModeIsActive() && Math.abs(rightEncoder.getCurrentPosition()) < Math.abs(targetCounts)) {
                setMotorPowers(0, power, 0, power); // Right side moves
                telemetry.addData("Target Encoder Counts", targetCounts);
                telemetry.addData("Right Encoder Position", rightEncoder.getCurrentPosition());
                telemetry.update();
            }
        } else {
            // Left side moves; right side is stationary
            while (opModeIsActive() && Math.abs(leftEncoder.getCurrentPosition()) < Math.abs(targetCounts)) {
                setMotorPowers(power, 0, power, 0); // Left side moves
                telemetry.addData("Target Encoder Counts", targetCounts);
                telemetry.addData("Left Encoder Position", leftEncoder.getCurrentPosition());
                telemetry.update();
            }
        }

        // Stop all motors after completing the turn
        setMotorPowers(0, 0, 0, 0);

        // Provide telemetry feedback
        telemetry.addData("Turn Complete", "Target Angle: %.2f degrees", targetAngle);
        telemetry.addData("Final Left Encoder Position", leftEncoder.getCurrentPosition());
        telemetry.addData("Final Right Encoder Position", rightEncoder.getCurrentPosition());
        telemetry.update();
    }

    public void singleSideTurn(double targetHeading, double power, boolean isLeftSideFixed) {
        double currentHeading = heading;  // Current heading from odometry
        double error = targetHeading - currentHeading;

        while (opModeIsActive() && Math.abs(error) > 1) {
            currentHeading = heading;  // Update the current heading
            error = targetHeading - currentHeading;

            // Normalize error to -180 to +180 degrees for shortest turn
            error = (error + 360) % 360;
            if (error > 180) {
                error -= 360;
            }

            if (isLeftSideFixed) {
                // Turn by moving only the right side
                setMotorPowers(0, power, 0, power);
            } else {
                // Turn by moving only the left side
                setMotorPowers(-power, 0, -power, 0);
            }

            // Debugging telemetry
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        // Stop the motors after reaching the target heading
        setMotorPowers(0, 0, 0, 0);
    }


    // Method to open the claw
    public void openClaw() {
        claw.setPosition(0.4);  // Fully open the claw
        telemetry.addData("Claw", "Open");
        telemetry.update();
    }

    // Method to close the claw
    public void closeClaw() {
        claw.setPosition(1.3);  // Fully close the claw
        telemetry.addData("Claw", "Close");
        telemetry.update();
    }


    // Method to rotate the wrist with a specific power for a set time
    public void rotateWrist(double position, double timeSeconds) {
        wrist.setPosition(position);  // Set power to rotate in the desired direction
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < timeSeconds) {
            telemetry.addData("Wrist Power", position);
            telemetry.addData("Time Remaining", "%.2f seconds", timeSeconds - timer.seconds());
            telemetry.update();
        }
        wrist.setPosition(0.13);  // Stop the wrist after the time has passed
    }

    public void turnInPlace(double targetAngle, double power) {

        lFront = hardwareMap.get(DcMotor.class, "lFront");
        rFront = hardwareMap.get(DcMotor.class, "rFront");
        lBack = hardwareMap.get(DcMotor.class, "lBack");
        rBack = hardwareMap.get(DcMotor.class, "rBack");

        lFront.setDirection(DcMotor.Direction.REVERSE);
        rFront.setDirection(DcMotor.Direction.FORWARD);
        lBack.setDirection(DcMotor.Direction.REVERSE);
        rBack.setDirection(DcMotor.Direction.FORWARD);


        lFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Constants (adjust for your robot)
        double TRACK_WIDTH = 15.0; // Distance between left and right wheels (inches)
        double WHEEL_DIAMETER = 1.8897637795; // Wheel diameter (inches)
        double TICKS_PER_REVOLUTION = 2000; // Motor ticks per revolution 353.68
        double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double TICKS_PER_INCH = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

        // Calculate rotation distance and ticks per degree
        double rotationDistance = Math.PI * TRACK_WIDTH;
        double ticksPerDegree = (TICKS_PER_INCH * rotationDistance) / 360.0;

        // Calculate target ticks for the desired angle
        int targetTicks = (int) (ticksPerDegree * targetAngle);

        // Reset encoders
        lFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions
        lFront.setTargetPosition(-targetTicks); // Move left motors backward
        lBack.setTargetPosition(-targetTicks);
        rFront.setTargetPosition(targetTicks); // Move right motors forward
        rBack.setTargetPosition(targetTicks);

        // Set to RUN_TO_POSITION
        lFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set motor power
        lFront.setPower(-power);
        lBack.setPower(-power);
        rFront.setPower(power);
        rBack.setPower(power);

        // Wait until all motors reach their target
        while (lFront.isBusy() && lBack.isBusy() &&
                rFront.isBusy() && rBack.isBusy()) {
            // Optionally, add telemetry or logging here
        }

        // Stop motors
        lFront.setPower(0);
        lBack.setPower(0);
        rFront.setPower(0);
        rBack.setPower(0);

        // Set motors back to RUN_WITHOUT_ENCODER
        lFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Method to turn the robot to a specific heading
    public void turnToHeading(double targetHeading, double power) {
        while (opModeIsActive() && Math.abs(heading - targetHeading) > 2) {
            double error = targetHeading - heading;

            // Scale power based on error
            double turnPower = Math.max(0.1, Math.min(power, Math.abs(error / 180.0)));
            if (error < 0) {
                turnPower = -turnPower;  // Adjust direction of the turn
            }

            // Apply motor powers for turning in place
            setMotorPowers(turnPower, -turnPower, turnPower, -turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", heading);
            telemetry.update();
        }
        setMotorPowers(0, 0, 0, 0);  // Stop the robot once the heading is reached
    }

    // Method to lift the arm to a desired height (in inches)
    public void liftArmToHeight(double targetHeightInches, double power, double holdTimeSeconds) {
        // Convert height in inches to encoder counts
        int targetPosition = (int)(targetHeightInches * COUNTS_PER_INCH_ARM);

        // Move arm to target position using the encoder
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);

        // Wait until the arm reaches the desired position
        while (opModeIsActive() && arm.isBusy()) {
            telemetry.addData("Target Height", targetHeightInches);
            telemetry.addData("Current Position", arm.getCurrentPosition());
            telemetry.update();
        }

        // Hold the arm in place for the specified amount of time
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // Keep motor power on to maintain the position
        arm.setPower(0.2);  // Low but sufficient power to hold position (adjust if needed)

        while (opModeIsActive() && holdTimer.seconds() < holdTimeSeconds) {
            telemetry.addData("Holding position", "Time remaining: %.2f seconds", holdTimeSeconds - holdTimer.seconds());
            telemetry.addData("Current Position", arm.getCurrentPosition());
            telemetry.update();
        }

        // Keep the arm holding the position after the hold period
        // Maintain RUN_TO_POSITION mode and apply a slightly higher power to hold position indefinitely
        arm.setPower(0.2);  // Continue applying a small power to hold position
    }

    // Set power to the drive motors (example layout for 4-wheeled robot)
    private void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        // Assuming motors are named in hardwareMap as "frontLeft", "frontRight", etc.
        hardwareMap.get(DcMotor.class, "lFront").setPower(-frontLeft);
        hardwareMap.get(DcMotor.class, "rFront").setPower(frontRight);
        hardwareMap.get(DcMotor.class, "lBack").setPower(-backLeft);
        hardwareMap.get(DcMotor.class, "rBack").setPower(backRight);
    }

    // Odometry update thread
    class OdometryGlobalPositionUpdate implements Runnable {
        private DcMotor leftEncoder, rightEncoder, horizontalEncoder;
        private double COUNTS_PER_INCH;

        private int previousLeftPosition = 0;
        private int previousRightPosition = 0;
        private int previousHorizontalPosition = 0;

        public OdometryGlobalPositionUpdate(DcMotor leftEncoder, DcMotor rightEncoder, DcMotor horizontalEncoder, double COUNTS_PER_INCH) {
            this.leftEncoder = leftEncoder;
            this.rightEncoder = rightEncoder;
            this.horizontalEncoder = horizontalEncoder;
            this.COUNTS_PER_INCH = COUNTS_PER_INCH;
        }

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                // Calculate encoder deltas
                int currentLeftPosition = leftEncoder.getCurrentPosition();
                int currentRightPosition = rightEncoder.getCurrentPosition();
                int currentHorizontalPosition = horizontalEncoder.getCurrentPosition();

                int deltaLeft = -currentLeftPosition + previousLeftPosition;
                int deltaRight = -currentRightPosition + previousRightPosition;
                int deltaHorizontal = -currentHorizontalPosition + previousHorizontalPosition;

                // Update previous positions
                previousLeftPosition = currentLeftPosition;
                previousRightPosition = currentRightPosition;
                previousHorizontalPosition = currentHorizontalPosition;

                // Calculate movement based on encoder changes
                double deltaVertical = (deltaLeft + deltaRight) / 2.0;
                double deltaHorizontalInches = deltaHorizontal / COUNTS_PER_INCH;

                // Update global position
                xPos += deltaHorizontalInches;  // Lateral movement
                // yPos -= deltaVertical / COUNTS_PER_INCH;
                yPos += deltaVertical / COUNTS_PER_INCH;  // Forward/backward movement

                // Update heading based on encoder difference
                double deltaHeading = (deltaRight - deltaLeft) / (ROBOT_DIAMETER * COUNTS_PER_INCH) * (360 / (2 * Math.PI));
                heading += deltaHeading;

                // Normalize heading to be within 0 to 360 degrees
                heading = (heading + 360) % 360;
            }
        }
    }
}
