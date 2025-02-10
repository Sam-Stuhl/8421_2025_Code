package org.firstinspires.ftc.teamcode.LastGame2024;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "ArmTesting", group = "Autonomous")
public class Armtest extends LinearOpMode {

    private DcMotor motor;
    private static final int TARGET_POSITION = 1000; // Adjust this value to your target position
    private static final int MOTOR_POWER = 1; // Motor power level (0.0 - 1.0)
    private static final int HOLD_POSITION = 5; // Tolerance for holding position (in counts)

    @Override
    public void runOpMode() {
        // Initialize the motor
        motor = hardwareMap.get(DcMotor.class, "Arm"); // Replace "motor" with your motor's name
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set motor to run to a specific position

        // Wait for the start button to be pressed
        waitForStart();

        // Set target position
        motor.setTargetPosition(TARGET_POSITION);

        // Set the motor power to move to the target position
        motor.setPower(MOTOR_POWER);

        // Wait until the motor reaches the target position
        while (opModeIsActive() && motor.isBusy()) {
            // Optional: Display the current position
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.update();
        }

        // Stop the motor once the target position is reached
        motor.setPower(0);

        // Hold the motor at the target position
        while (opModeIsActive()) {
            // Check if the motor is within the holding tolerance
            int currentPosition = motor.getCurrentPosition();
            if (Math.abs(currentPosition - TARGET_POSITION) > HOLD_POSITION) {
                // If out of range, apply a small correction
                motor.setPower(MOTOR_POWER * (currentPosition < TARGET_POSITION ? 0.1 : -0.1)); // Adjust power to hold
            } else {
                motor.setPower(0); // Hold position
            }

            // Optional: Display the current position
            telemetry.addData("Current Position", currentPosition);
            telemetry.update();
        }

        // Reset motor mode and power when done
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
}