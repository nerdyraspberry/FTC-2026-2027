package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name="Base TeleOp")
public class BaseTeleOp extends LinearOpMode {
    private final int CAMERA_vID = 0x046D;
    private final int CAMERA_pID = 0x0892;

    final int CAM_X_RESOLUTION = 640; // The default of the vision processor

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private MecanumDrive drivetrain;

    private String[] targetedCodes;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        this.initDriveTrain();
        this.initTargetingScanner();

        this.targetedCodes = new String[]{"RED AUDIENCE", "RED SCORING"};

        boolean manualSteering = true;
        double speedLimit = 1.0;

        boolean debounceA = false;
        boolean debounceB = false;

//        AndroidTextToSpeech tts = new AndroidTextToSpeech();
//        tts.initialize();

        waitForStart();
        while (opModeIsActive()) {
            // April Tag testing
            List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            double targetingScannerRotation = 0.0;

            for (AprilTagDetection detection : currentDetections) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                    if (singleDet.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));

                        // Test: turn to buffer
                        double relativeTurn = 2.0*(singleDet.center.x / (double)this.CAM_X_RESOLUTION) - 1.0;
                        final double MARGIN = 0.1;
                        if (relativeTurn < -MARGIN || relativeTurn > MARGIN) {
                            targetingScannerRotation = .7 * relativeTurn;
                        } else {
                            targetingScannerRotation = 0.0;
                        }
                    }
                } else {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    boolean tagUsed = false;
                    for (String usedTag : this.targetedCodes) {
                        if (clusterDet.metadata.name.equalsIgnoreCase(usedTag)) {
                            tagUsed = true;
                            break;
                        }
                    }

                    if (tagUsed) {
                        // Test: turn to buffer
                        double relativeTurn = -0.02 * detection.ftcPose.bearing;
                        final double MARGIN = 0.1;
                        if (relativeTurn < -MARGIN || relativeTurn > MARGIN) {
                            targetingScannerRotation = relativeTurn;
                        } else {
                            targetingScannerRotation = 0.0;
                        }
                        System.out.println(targetingScannerRotation);
                    }
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            // Gamepad mode
            if (gamepad1.x) {
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

            sleep(20);
        }

        // Tear down vision portal
        this.visionPortal.close();
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

    private void initTargetingScanner() {
        // For camera information: see ConceptAprilTag in the sample.
//        CameraCompatibilityManager.getInstance()
//                .addQuirk(this.CAMERA_vID, this.CAMERA_pID, CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE);

        this.aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                // The 36h11 is the only tag family used by the FTC competition
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // If this is not done, it will detect tags incorrectly
                // .setTagLibrary(new AprilTagLibrary.Builder().build())
                .build();

        VisionPortal.Builder visBuilder = new VisionPortal.Builder();
        // Add camera
        visBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        // Enable camera streaming to driver station (this is not allowed
        // in matches, I believe)
        visBuilder.enableLiveView(true);
        // Add April Tag processor
        visBuilder.addProcessor(this.aprilTag);

        this.visionPortal = visBuilder.build();
    }
}
