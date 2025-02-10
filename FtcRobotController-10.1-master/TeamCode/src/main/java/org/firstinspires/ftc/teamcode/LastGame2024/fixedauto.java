package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "fixedauto", group = "Linear Opmode")
public class fixedauto extends LinearOpMode {

    // Odometry wheel motors
    DcMotor leftEncoder, rightEncoder, horizontalEncoder;

    // Robot parameters
    final double COUNTS_PER_INCH = 353.68; // Adjust based on your robot's specifics
    final double ROBOT_DIAMETER = 15.0;    // Example robot diameter in inches

    // Position variables
    private double xPos = 0, yPos = 0;     // X, Y coordinates in inches
    private double heading = 0;            // Heading in degrees

    // Timer for loop
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize hardware map
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

        // Wait for the start command
        waitForStart();

        // Start the update odometry thread
        Thread positionUpdate = new Thread(new OdometryGlobalPositionUpdate(leftEncoder, rightEncoder, horizontalEncoder, COUNTS_PER_INCH));
        positionUpdate.start();

        // Autonomous movement example
        moveToPosition(24, 24, 0.5); // Move to (24, 24)
        turnToHeading(90, 0.4);      // Turn to 90 degrees
        moveToPosition(-12, 0, 0.5); // Move left by 12 inches

        // Stop the position update thread
        positionUpdate.interrupt();
    }

    // Method to move the robot to a given X, Y position
    public void moveToPosition(double targetXInches, double targetYInches, double power) {
        ElapsedTime moveTimer = new ElapsedTime();
        moveTimer.reset();

        double minPower = 0.1; // Minimum power to prevent stalling
        double maxPower = 0.5; // Maximum power for movement

        while (opModeIsActive() && moveTimer.seconds() < 10) {
            double distanceToX = targetXInches - xPos;
            double distanceToY = targetYInches - yPos;

            double distance = Math.hypot(distanceToX, distanceToY);
            if (distance < 2) break; // Stop if close enough to the target

            double scaledPower = Math.max(minPower, Math.min(maxPower, distance / 10));

            double xPower = (distanceToX / distance) * scaledPower;
            double yPower = (distanceToY / distance) * scaledPower;

            setMotorPowers(yPower + xPower, yPower - xPower, yPower - xPower, yPower + xPower);

            telemetry.addData("Target X", targetXInches);
            telemetry.addData("Target Y", targetYInches);
            telemetry.addData("Current X", xPos);
            telemetry.addData("Current Y", yPos);
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
        setMotorPowers(0, 0, 0, 0); // Stop the robot
    }

    // Method to turn the robot to a specific heading
    public void turnToHeading(double targetHeading, double power) {
        ElapsedTime turnTimer = new ElapsedTime();
        turnTimer.reset();

        while (opModeIsActive() && turnTimer.seconds() < 5) {
            double error = targetHeading - heading;
            error = (error + 360) % 360; // Normalize error to [0, 360]
            if (error > 180) error -= 360; // Adjust to [-180, 180]

            if (Math.abs(error) < 2) break; // Stop if close enough to the target heading

            double turnPower = Math.max(0.1, Math.min(power, Math.abs(error / 180.0)));
            turnPower *= Math.signum(error); // Adjust direction

            setMotorPowers(turnPower, -turnPower, turnPower, -turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", heading);
            telemetry.addData("Error", error);
            telemetry.update();
        }
        setMotorPowers(0, 0, 0, 0); // Stop the robot
    }

    // Set power to the drive motors
    private void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
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
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                int currentLeftPosition = leftEncoder.getCurrentPosition();
                int currentRightPosition = rightEncoder.getCurrentPosition();
                int currentHorizontalPosition = horizontalEncoder.getCurrentPosition();

                int deltaLeft = currentLeftPosition - previousLeftPosition;
                int deltaRight = currentRightPosition - previousRightPosition;
                int deltaHorizontal = currentHorizontalPosition - previousHorizontalPosition;

                previousLeftPosition = currentLeftPosition;
                previousRightPosition = currentRightPosition;
                previousHorizontalPosition = currentHorizontalPosition;

                double deltaVertical = (deltaLeft + deltaRight) / 2.0;
                double deltaHorizontalInches = deltaHorizontal / COUNTS_PER_INCH;


                xPos += deltaHorizontalInches;
                //yPos += deltaVertical / COUNTS_PER_INCH;

                double deltaHeading = (deltaRight - deltaLeft) / (ROBOT_DIAMETER * COUNTS_PER_INCH) * (360 / (2 * Math.PI));
                heading = (heading + deltaHeading + 360) % 360;
            }
        }
    }
}
