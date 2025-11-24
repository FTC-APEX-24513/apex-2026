package edu.exeter.apex.ftc.teamcode.subsystems;

import static pedroPathing.Tuning.follower;

import android.util.Size;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.List;
import pedroPathing.Constants;

public class LimelightJava {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);
        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) return;

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    public AprilTagDetection getTagById(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
    public void localize() {

        if (detectedTags.isEmpty()) return;
        AprilTagDetection tag = detectedTags.get(0);

        if (tag.ftcPose == null) return;

        if(tag.ftcPose != null){
            double offsetX = tag.ftcPose.x;
            double offsetY = tag.ftcPose.y;
            double offsetHeading = tag.ftcPose.yaw;

            Pose ftcRelativePose = new Pose(offsetX, offsetY, offsetHeading);
            Pose relativePose = ftcRelativePose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            Pose targetPose = new Pose(0, 39.3701, 0);
            Pose blueScore = new Pose();

            follower.setPose(relativePose);
            follower.followPath(
                    follower.pathBuilder()
                    .addPath(new BezierLine(relativePose, targetPose))
                    .setLinearHeadingInterpolation(relativePose.getHeading(), targetPose.getHeading())
                            .build()
            );

            follower.setPose(blueScore);
        }
    }
}