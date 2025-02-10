package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Odometry2", group = "Linear Opmode")
public class Odometry2 extends LinearOpMode {

    private Servo wrist = null;

    private Servo claw = null;

    private DcMotor arm = null;

    private DcMotor lFront = null;
    private DcMotor rFront = null;
    private DcMotor lBack = null;
    private DcMotor rBack = null;

    private Servo joint = null;

    private CRServo R = null;
    private CRServo L = null;



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
    private static final double WRISTUP = 0.373;
    private static final double WRISTDOWN = 0.95;
    double wristValue = WRISTUP;

    // Timer for loop
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize hardware map

        arm = hardwareMap.get(DcMotor.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        R = hardwareMap.get(CRServo.class, "R");
        L = hardwareMap.get(CRServo.class, "L");
        joint = hardwareMap.get(Servo.class, "joint");

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
        joint.setPosition(1);


        // Wait for the start command
        waitForStart();

        // Start the update odometry thread
        Thread positionUpdate = new Thread(new OdometryGlobalPositionUpdate(leftEncoder, rightEncoder, horizontalEncoder, COUNTS_PER_INCH));
        positionUpdate.start();
        moveToPosition(0, 0, 1);
        waitbot(0.1);
        moveToPosition(0, 8, 1);
        waitbot(0.1);
        moveToPosition(-12, 8, 1);
        waitbot(0.1);
        moveToPosition(-12, 6, 1);
        moveArmByTime(1, 3);
        moveToPosition(-12, 1, 1);
        rotateWrist(.37, 2.0);
        rotatejoint(0.75, 1);
        outake(1);
        rotatejoint(0.50, 1);
        rotateWrist(0.716, 3);
        moveArmByTime(-1, 2);
        intakepower(0.5);
        moveToPosition(-8, -2, 0.5);
        waitbot(1);
        moveToPosition(-7, 6, 1);
        waitbot(0.1);
        moveToPosition(-12, -2, 1);
        intakepower(0);
        moveArmByTime(1, 2.5);
        rotateWrist(.37, 3.0);
        rotatejoint(0.762, 1);
        outakepower(0.8, 1);
        rotatejoint(0.50, 1);
        rotateWrist(0.716, 3);
        moveArmByTime(-1, 2);
        positionUpdate.interrupt();
    }

    public void moveArmByTime(double power, double seconds) {

       // wrist.setPosition(wristposition);
        // Set the arm motor power
        arm.setPower(power);
        // Start a timer
        ElapsedTime timer = new ElapsedTime();
        timer.reset();


        // Keep the motor running for the specified time
        while (opModeIsActive() && timer.seconds() < seconds) {
            telemetry.addData("Arm Power", power);
            telemetry.addData("Time Remaining", "%.2f seconds", seconds - timer.seconds());
            telemetry.update();
        }
        arm.setPower(0.02);
    }
    public void moveArmAndWrist(double armpower, double wristPosition, double wristTimeSeconds) {
        // Lift the arm to the specified height

        arm.setPower(armpower); // Adjust power as needed

        // Start wrist rotation
        wrist.setPosition(wristPosition);  // Set wrist to the desired position
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && (arm.isBusy() || timer.seconds() < wristTimeSeconds)) {
            // Update telemetry for arm and wrist concurrently
            if (arm.isBusy()) {
                telemetry.addData("Arm Position", arm.getCurrentPosition());
            } else {
                arm.setPower(0.02);
            }

            if (timer.seconds() < wristTimeSeconds) {
                telemetry.addData("Wrist Position", wristPosition);
                telemetry.addData("Wrist Time Remaining", "%.2f seconds", wristTimeSeconds - timer.seconds());
            }

            telemetry.update();
        }

        arm.setPower(0.02); // Minimal power to hold the arm position
        //wrist.setPosition(0.50); // Maintain wrist position
    }
    // Example of the liftArmToHeight method (if not already defined)
    public void liftArmToHeight(double height) {
        // Example: Set motor power or encoder target to reach the desired height
        arm.setTargetPosition((int)(height * COUNTS_PER_INCH)); // Replace COUNTS_PER_INCH with your actual value
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0); // Adjust power as needed

        while (arm.isBusy()) {
            // Wait until the arm reaches the target position
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.update();
        }

        arm.setPower(0.02); // Stop motor after reaching the target
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
    public void intake(double timeSeconds) {
        R.setPower(1);
        L.setPower(-1);  // Fully open the claw
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeSeconds) {
            telemetry.update();
        }
        R.setPower(0);
        L.setPower(0);
    }
    public void intakepower(double power) {
        R.setPower(power);
        L.setPower(-power);  // Fully open the claw
    }

    public void outakepower(double power, double timeSeconds) {
        R.setPower(-power);
        L.setPower(power);  // Fully open the claw
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeSeconds) {
            telemetry.update();
        }
        R.setPower(0);
        L.setPower(0);
    }

    // Method to close the claw
    public void outake(double timeSeconds) {
        R.setPower(-0.3);
        L.setPower(0.3);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeSeconds) {
            telemetry.update();
        }
        R.setPower(0);
        L.setPower(0);
    }
    public void waitbot(double timeSeconds) {
        hardwareMap.get(DcMotor.class, "lFront").setPower(0);
        hardwareMap.get(DcMotor.class, "rFront").setPower(0);
        hardwareMap.get(DcMotor.class, "lBack").setPower(0);
        hardwareMap.get(DcMotor.class, "rBack").setPower(0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeSeconds) {
            telemetry.update();
        }
    }

    public void rotatejoint(double position, double timeSeconds) {
        joint.setPosition(position);  // Set power to rotate in the desired direction
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < timeSeconds) {
            telemetry.addData("Wrist Power", position);
            telemetry.addData("Time Remaining", "%.2f seconds", timeSeconds - timer.seconds());
            telemetry.update();
        }
        joint.setPosition(position);  // Stop the wrist after the time has passed
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
        wrist.setPosition(position);  // Stop the wrist after the time has passed
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
