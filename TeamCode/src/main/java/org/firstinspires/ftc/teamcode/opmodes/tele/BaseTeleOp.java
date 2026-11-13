package org.firstinspires.ftc.teamcode.opmodes.tele;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.data.MecanumConfig;
import org.firstinspires.ftc.teamcode.kowalski.autoshooter.AutoShooter;
import org.firstinspires.ftc.teamcode.kowalski.Debouncer;
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
        // Initialise hardware
        this.initDriveTrain();
        this.targetingScanner = new TargetingScanner(hardwareMap);
        this.targetingScanner.scanFor = this.targetedCodes;
        this.imu = new IMUWrapper(hardwareMap);

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
//        DcMotor largeOuttake = hardwareMap.get(DcMotor.class, "largeOuttake");
        DcMotor smallOuttake = hardwareMap.get(DcMotor.class, "smallOuttake");
        Servo hatch = hardwareMap.get(Servo.class, "hatch");
        // Variables
        boolean manualSteering = true;
        boolean intakeRunning = false;
        double speedLimit = 1.0;

        Debouncer debounceDown = new Debouncer();
        Debouncer debounceUp = new Debouncer();
        Debouncer debounceIntake = new Debouncer();
        Debouncer debouncePrev = new Debouncer();
        Debouncer debounceNext = new Debouncer();

        double lastTagDirection = 0.0;
        double lastTagSeenAt = -1.0;

        double targetingScannerRotation = 0.0;

//        AutoShooter largeAutoShooter = new AutoShooter(largeOuttake, hatch);
        AutoShooter smallAutoShooter = new AutoShooter(smallOuttake, hatch);

        // Main loop
        waitForStart();
        while (opModeIsActive()) {
            double time = getRuntime();

            // Targeting scanner / AprilTag detector
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
            // Gamepad mode
            // - Mode configuration
            if (gamepad1.x || Math.abs(gamepad1.right_stick_x) > .01)
                manualSteering = true;
            if (gamepad1.y)
                manualSteering = false;
            // - Speed limiter
            if (debounceDown.wasJustPressed(gamepad1.dpad_down))
                speedLimit = Math.max(0.0, speedLimit - .1);
            if (debounceUp.wasJustPressed(gamepad1.dpad_up))
                speedLimit = Math.min(speedLimit + .1, 1.0);
            // - Intake / outtake control
            if (debounceIntake.wasJustPressed(gamepad1.a))
                intakeRunning = !intakeRunning;
            intake.setPower(intakeRunning ? 1.0 : 0.0);
            if (gamepad1.dpad_left)
                smallAutoShooter.start(time);
            if (gamepad1.b) {
                if (smallAutoShooter.isRunning()) smallAutoShooter.abort();
//                if (largeAutoShooter.isRunning()) largeAutoShooter.abort();
            }

            if (debouncePrev.wasJustPressed(gamepad1.left_trigger_pressed)) {
                // Cycle back
                smallAutoShooter.setState(
                        smallAutoShooter.getState().back()
                );
            }
            if (debounceNext.wasJustPressed(gamepad1.right_trigger_pressed)) {
                // Cycle forward
                smallAutoShooter.setState(
                        smallAutoShooter.getState().next()
                );
            }
            // - - Update intake/outtake motors
//            largeAutoShooter.tick(time);
            smallAutoShooter.tick(time);
            telemetry.addData("Outtake Mode", smallAutoShooter.getState());
            // - Mode telemetry
            telemetry.addData("Mode", manualSteering ? "Manual Steering" : "Targeting Scanner");
            telemetry.addData("Speed limit", speedLimit);
            telemetry.update();
            // Configure the drivetrain
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
