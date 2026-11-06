package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumMotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class BaseTeleOp extends LinearOpMode {
    /*private final int CAMERA_vID = 0x046D;
    private final int CAMERA_pID = 0x0892;

    final int CAM_X_RESOLUTION = 640; // The default of the vision processor */

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private MecanumDrive drivetrain;

    private String[] targetedCodes;

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
        this.initTargetingScanner();
        // Variables
        boolean manualSteering = true;
        double speedLimit = 1.0;

        boolean debounceA = false;
        boolean debounceB = false;

        double lastTagDirection = 0.0;
        double lastTagSeenAt = -1.0;

//        AndroidTextToSpeech tts = new AndroidTextToSpeech();
//        tts.initialize();
        // Main loop
        waitForStart();
        // TODO: split out targeting scanner into separate class
        while (opModeIsActive()) {
            // April Tag testing
            List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            double targetingScannerRotation = 0.0;

            for (AprilTagDetection detection : currentDetections) {
                if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                    boolean tagUsed = false;
                    for (String usedTag : this.targetedCodes) {
                        if (clusterDet.metadata.name.equalsIgnoreCase(usedTag)) {
                            tagUsed = true;
                            break;
                        }
                    }

                    // TODO: check if the tag belongs to the currently active (raised) hive

                    if (tagUsed) {
                        // Test: turn to buffer
                        double relativeTurn = -0.02 * detection.ftcPose.bearing;
                        final double MARGIN = 0.1;
                        if (relativeTurn < -MARGIN || relativeTurn > MARGIN) {
                            targetingScannerRotation = relativeTurn;

                            // TODO: only memorise if it is in the edge of the camera FOV
                            lastTagSeenAt = getRuntime();
                            lastTagDirection = relativeTurn < 0.0 ? -1.0 : 1.0;
                        } else {
                            targetingScannerRotation = 0.0;
                        }
                        System.out.println(targetingScannerRotation);
                    }
                }
            }   // end for() loop

            // Rotate to where we previously saw a tag if we are in targeting scanner mode
            // but no tag is seen.  If the previously seen tag data is too old, we purge it.
            if (getRuntime() - lastTagSeenAt > 2.0)
                lastTagDirection = 0.0;
            telemetry.addData("Original targetingScannerRotation", targetingScannerRotation);
            if (targetingScannerRotation == 0.0 && lastTagDirection != 0.0)
                targetingScannerRotation = lastTagDirection;
            telemetry.addData("New targetingScannerRotation", targetingScannerRotation);
            telemetry.addData("Current t", getRuntime());
            telemetry.addData("Last tag direction", lastTagDirection);
            telemetry.addData("Last tag time", lastTagSeenAt);

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
