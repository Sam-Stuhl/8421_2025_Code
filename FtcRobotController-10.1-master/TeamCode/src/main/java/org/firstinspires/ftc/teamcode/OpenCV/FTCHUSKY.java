package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
@Autonomous(name = "Sensor: HuskyLens", group = "Sensor")
public class FTCHUSKY extends LinearOpMode {


    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);

        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }

            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }

            for (int i = 0; i < blocks.length; i++) {
                int x = blocks[i].x;
                int y = blocks[i].y;

                telemetry.addData("Object ",+ (i + 1) + ": X = " + x + ", Y = " + y);

                // Categorize based on position
                if (x < 100) {
                    telemetry.addData(">>", "Object is on the left.");
                    telemetry.update();
                } else if (x > 200) {
                    telemetry.addData(">>", "Object is on the right.");
                    telemetry.update();
                } else {
                    telemetry.addData(">>", "Object is in the center.");
                    telemetry.update();
                }

                if (y < 100) {
                    telemetry.addData(">>", "Object is at the top.");
                    telemetry.update();
                } else if (y > 200) {
                    telemetry.addData(">>", "Object is at the bottom.");
                    telemetry.update();
                } else {
                    telemetry.addData(">>", "Object is in the middle vertically.");
                    telemetry.update();
                }
            }
        }
    }
    }
