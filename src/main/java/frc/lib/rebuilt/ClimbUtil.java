package frc.lib.rebuilt;

import java.util.Set;

import frc.lib.math.AllianceFlipUtil;
import frc.robot.FieldConstants;

import edu.wpi.first.math.geometry.Pose2d;

public final class ClimbUtil {
    private ClimbUtil() {}

    public static Pose2d getPreClimbPose(Pose2d robotPose) {
        return
            AllianceFlipUtil.apply(
                robotPose.nearest(Set.of(FieldConstants.Tower.blueLeftPreClimbPose, FieldConstants.Tower.blueRightPreClimbPose)));
    }

    public static Pose2d getClimbPose(Pose2d robotPose) {
        return
            AllianceFlipUtil.apply(
                robotPose.nearest(Set.of(FieldConstants.Tower.blueLeftClimbPose, FieldConstants.Tower.blueRightClimbPose)));
    }
}