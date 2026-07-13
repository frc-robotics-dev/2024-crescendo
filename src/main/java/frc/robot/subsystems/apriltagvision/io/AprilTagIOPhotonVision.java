package frc.robot.subsystems.apriltagvision.io;

import java.util.List;
import java.util.Optional;

import frc.robot.FieldConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.CameraConfig;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class AprilTagIOPhotonVision implements AprilTagIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public AprilTagIOPhotonVision(CameraConfig config) {
        camera = new PhotonCamera(config.name());
        poseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, config.robotToCamera());
    }

    @Override
    public void updateInputs(AprilTagIOInputs inputs) {
        Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

        for (var result : camera.getAllUnreadResults()) {
            estimatedRobotPose = poseEstimator.estimateCoprocMultiTagPose(result);

            if (estimatedRobotPose.isEmpty()) {
                estimatedRobotPose = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            estimatedRobotPose.ifPresent(
                est -> {
                    inputs.timestamp = est.timestampSeconds;
                    inputs.estimatedRobotPose = est.estimatedPose;
                });

            inputs.estimatedStdDevs = getEstimationStdDevs(estimatedRobotPose, result.getTargets());
        }
    }

    private Matrix<N3, N1> getEstimationStdDevs(Optional<EstimatedRobotPose> estimate, List<PhotonTrackedTarget> targets) {
        if (estimate.isEmpty()) {
            return AprilTagVisionConstants.kSingleTagStdDevs;
        }
        
        var estStdDevs = AprilTagVisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double averageDist = 0;

        for (var target : targets) {
            var tagPose =
                FieldConstants.aprilTagFieldLayout.getTagPose(target.getFiducialId());

            if (tagPose.isEmpty()) {
                continue;
            }

            numTags++;

            averageDist +=
                tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimate.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            return AprilTagVisionConstants.kSingleTagStdDevs;
        }

        averageDist /= numTags;

        if (numTags > 1) {
            estStdDevs = AprilTagVisionConstants.kMultiTagStdDevs;
        }

        if (numTags == 1 && averageDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (averageDist * averageDist / 30));
        }
        
        return estStdDevs;
    }
}