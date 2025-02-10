package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(name="FTCRR", group="Robot")
public class FTCRR extends LinearOpMode {

    public DcMotor intake = null;



    public DcMotor extendo = null;
    public Servo launcher = null;

    public Servo bucket = null;

    public Servo bucketarm = null;

    private Servo bucketR = null;

    private Servo bucketL = null;

    @Override
    public void runOpMode() throws InterruptedException {
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        bucket = hardwareMap.get(Servo.class, "bucket");
        bucketarm = hardwareMap.get(Servo.class, "bucketarm");
        bucketR = hardwareMap.get(Servo.class, "bucketR");
        bucketL = hardwareMap.get(Servo.class, "bucketL");


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(11.38, 66.97, Math.toRadians(267.71)))
                .splineTo(new Vector2d(13.06, 27.31), Math.toRadians(270.00))
                .splineTo(new Vector2d(47.14, 32.75), Math.toRadians(-1.59))
                .splineTo(new Vector2d(29.12, 32.75), Math.toRadians(180.00))
                .splineTo(new Vector2d(33.45, 56.78), Math.toRadians(5.71))
                .splineTo(new Vector2d(60.13, 61.39), Math.toRadians(5.91))
                .build();


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectorySequence(untitled0);
        }

    }
    }