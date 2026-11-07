package org.firstinspires.ftc.teamcode.opmodes.tele;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.kowalski.ScannerResult;
import org.firstinspires.ftc.teamcode.kowalski.TargetingScanner;
import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class BaseTeleOp extends LinearOpMode {
    /*private final int CAMERA_vID = 0x046D;
    private final int CAMERA_pID = 0x0892;

    final int CAM_X_RESOLUTION = 640; // The default of the vision processor */

    private TargetingScanner targetingScanner;

    private MecanumDrive drivetrain;

    private String[] targetedCodes;

    private IMU imu;

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
        initIMU();
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
        this.drivetrain
                .setSigns(MecanumMotor.FrontLeft, "-", "-", "+")
                .setSigns(MecanumMotor.FrontRight, "-", "+", "-")
                .setSigns(MecanumMotor.BackLeft, "+", "-", "-")
                .setSigns(MecanumMotor.BackRight, "+", "+", "+");
    }

    private void initIMU() {
        // de IMU initializeren
        this.imu = hardwareMap.get(IMU.class, "imu");
        // TODO bepaal orientatie en zet de juiste dingen neer bij logo en usb directie
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    // mag alleen gebruikt worden nadat intitIMU()
    private double getYaw() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    // mag alleen gebruikt worden nadat intitIMU()
    private void resetYaw() {
        this.imu.resetYaw();
    }
}
