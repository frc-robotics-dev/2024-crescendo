package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

    public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
    public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Pose2d getTagPose2d(int tagId) {
        return
            aprilTagFieldLayout
                .getTagPose(tagId)
                .orElseThrow(() -> new IllegalArgumentException("No tag with ID " + tagId + " found in layout."))
                .toPose2d();
    }

    public static class Lines {
        public static final double blueInitLineX = Units.inchesToMeters(156.8); // Alliance wall To init line
    }
}