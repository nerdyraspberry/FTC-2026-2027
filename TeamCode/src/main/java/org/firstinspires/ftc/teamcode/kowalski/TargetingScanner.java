package org.firstinspires.ftc.teamcode.kowalski;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TargetingScanner {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    public String[] scanFor;

    public TargetingScanner(HardwareMap hardwareMap) {
        // For camera information: see ConceptAprilTag in the sample.
//        CameraCompatibilityManager.getInstance()
//                .addQuirk(this.CAMERA_vID, this.CAMERA_pID, CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE);

        this.aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                // The 36h11 is the only tag family used by the FTC competition
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                // If this is not done, it will detect tags incorrectly
                // .setTagLibrary(new AprilTagLibrary.Builder().build())
//                .setLensIntrinsics(622.001f, 622.001f, 319.803f, 241.251f)
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

    /** Perform the finalisation routine of the targeting scanner.
     * Currently this only closes the VisionPortal.
     */
    public void close() {
        this.visionPortal.close();
    }

    /** Scan the camera view for the desired AprilTag clusters. The set of clusters to scan for
     * can be set using @TargetingScanner.scanFor.
     * @return A scanner result, containing if any tags were found, and if so, their bearing.
     */
    public ScannerResult scan() {
        ScannerResult result = new ScannerResult();
        result.any = false;

        List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagClusterDetection) {
                // Get tags
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                // Check if this tag is used
                boolean tagUsed = false;
                for (String usedTag : this.scanFor) {
                    if (clusterDet.metadata.name.equalsIgnoreCase(usedTag)) {
                        tagUsed = true;
                        break;
                    }
                }
                // Return tag bearing if it is used
                if (tagUsed) {
                    result.any = true;
                    result.bearing = detection.ftcPose.bearing;
                    return result;
                }
            }
        }   // end for() loop

        return result;
    }
}
