package org.firstinspires.ftc.teamcode.opmodes.tele;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.MecanumConfig;
import org.firstinspires.ftc.teamcode.kowalski.IMUWrapper;
import org.firstinspires.ftc.teamcode.kowalski.ScannerResult;
import org.firstinspires.ftc.teamcode.kowalski.TargetingScanner;
import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;

public class BaseTeleOp extends LinearOpMode {
    private TargetingScanner targetingScanner;

    private MecanumDrive drivetrain;

    private String[] targetedCodes;

    private IMUWrapper imu;

    public void setAlliance(Alliance alliance) {
        // This OpMode can be executed as either belonging to the Red or the Blue alliance.
        // This is done by making two TeleOp classes, BlueTeleOp and RedTeleOp, which are just
        // a thin wrapper around BaseTeleOp, calling this function and then runOpMode.
        if (alliance == Alliance.Blue) {
            this.targetedCodes = new String[]{"BLUE AUDIENCE", "BLUE SCORING"};
        } else {
            this.targetedCodes = new String[]{"RED AUDIENCE", "RED SCORING"};
        }
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        // Sanity check: Check if the alliance has been configured. If not, the OpMode
        // was invoked incorrectly.
        if (this.targetedCodes.length == 0) {
            throw new IllegalStateException(
                    "OpMode is uitgevoerd zonder dat de alliantiekleur is ingesteld!"
            );
        }
        // Initialise drivetrain and scanner objects
        this.initDriveTrain();
        this.targetingScanner = new TargetingScanner(hardwareMap);
        this.targetingScanner.scanFor = this.targetedCodes;
        this.imu = new IMUWrapper(hardwareMap);
        // Variables
        boolean manualSteering = true;
        double speedLimit = 1.0;

        boolean debounceA = false;
        boolean debounceB = false;

        double lastTagDirection = 0.0;
        double lastTagSeenAt = -1.0;

        double targetingScannerRotation = 0.0;

//        AndroidTextToSpeech tts = new AndroidTextToSpeech();
//        tts.initialize();
        // Main loop
        waitForStart();
        // TODO: split out targeting scanner into separate class
        while (opModeIsActive()) {
            // April Tag testing
            // TODO: check if the tag belongs to the currently active (raised) hive

            ScannerResult scanRes = this.targetingScanner.scan();
            if (scanRes.any)
                telemetry.addData("BEARING TO TAG", scanRes.bearing);
            else
                telemetry.addLine("NO TAG SEEN");

            if (scanRes.any) {
                // Test: turn to buffer
                double relativeTurn = -0.02 * scanRes.bearing;
                final double MARGIN = 0.1;
                if (Math.abs(relativeTurn) > MARGIN) {
                    targetingScannerRotation = relativeTurn;
                } else {
                    targetingScannerRotation = 0.0;
                }
            }

            telemetry.addData("Original targetingScannerRotation", targetingScannerRotation);
            System.out.printf("TS rotation: %s %f\n",
                    scanRes.any ? "FOUND" : "not found", targetingScannerRotation);

            telemetry.addData("New targetingScannerRotation", targetingScannerRotation);
            telemetry.addData("Current t", getRuntime());
            telemetry.addData("Last tag direction", lastTagDirection);
            telemetry.addData("Last tag time", lastTagSeenAt);

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            // Gamepad mode
            if (gamepad1.x || Math.abs(gamepad1.right_stick_x) > .01) {
                manualSteering = true;
//                tts.speak("Manual steering activated.");
            }
            if (gamepad1.y) {
                manualSteering = false;
//                tts.speak("Targeting scanner activated.");
            }

            if (gamepad1.dpad_down) {
                if (!debounceA) {
                    speedLimit = Math.max(0.0, speedLimit - .1);
                    debounceA = true;
                }
            } else debounceA = false;
            if (gamepad1.dpad_up) {
                if (!debounceB) {
                    speedLimit = Math.min(speedLimit + .1, 1.0);
                    debounceB = true;
                }
            } else debounceB = false;


            telemetry.addData("Mode", manualSteering ? "Manual Steering" : "Targeting Scanner");
            telemetry.addData("Speed limit", speedLimit);
            telemetry.update();

            drivetrain
                    .move(gamepad1.left_stick_x * speedLimit, gamepad1.left_stick_y * speedLimit)
                    .rotate(manualSteering ? gamepad1.right_stick_x * speedLimit
                            : targetingScannerRotation)
                    .apply();

            // Decay TS rotation
            targetingScannerRotation *= 0.90;

            sleep(20);
        }

        // Tear down vision portal
        this.targetingScanner.close();
    }

    private void initDriveTrain() {
        this.drivetrain = new MecanumDrive(
                hardwareMap,
                "left forward", "left backward",
                "right forward", "right backward"
        );
        MecanumConfig.apply(this.drivetrain);
    }
}
