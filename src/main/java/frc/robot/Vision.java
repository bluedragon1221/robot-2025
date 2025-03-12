package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision {
    private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    private static class EstimatablePhotonCamera {
        PhotonCamera camera;
        PhotonPoseEstimator estimator;

        EstimatablePhotonCamera(String name, Transform3d cam_offset) {
            camera = new PhotonCamera(name);
            estimator = new PhotonPoseEstimator(FIELD_LAYOUT, POSE_STRATEGY, cam_offset);
        }
    }

    private final EstimatablePhotonCamera cameras[] = {
        new EstimatablePhotonCamera("front_camera", null),
        new EstimatablePhotonCamera("back_camera", null)
    };

    private static Vision instance;

    private Vision() {}

    public EstimatedRobotPose[] getCameraPoseEstimations() {
        EstimatedRobotPose[] transform = new EstimatedRobotPose[cameras.length];

        int camera_index = 0;
        for (EstimatablePhotonCamera camera : cameras) {
            for (PhotonPipelineResult result : camera.camera.getAllUnreadResults()) {
                if (!result.hasTargets()) continue;

                Optional<EstimatedRobotPose> pose = camera.estimator.update(result);
                if (pose.isPresent())
                    transform[camera_index] = pose.get();
            }
            camera_index++;
        }

        return transform;
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }

        return instance;
    }

}
