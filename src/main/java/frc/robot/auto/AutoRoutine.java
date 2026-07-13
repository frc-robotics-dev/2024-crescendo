package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.Path;

public abstract class AutoRoutine extends Command {
    public abstract Path[] getTrajectory();

    public Pose2d getStartPose() {
        return getTrajectory()[0].getStartPose();
    }

    public String getName() {
        return getClass().getSimpleName();
    }
}