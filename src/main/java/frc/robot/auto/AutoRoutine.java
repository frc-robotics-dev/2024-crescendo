package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.Path;

public interface AutoRoutine {
    public Command getCommand();
    public Path[] getTrajectory();

    public default Pose2d getStartPose() {
        return getTrajectory()[0].getStartPose();
    }

    public default String getName() {
        return getClass().getSimpleName();
    }
}