package frc.robot.auto.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.auto.pathplanner.PathPlannerUtil;
import frc.robot.auto.AutoRoutine;
import frc.robot.commands.FollowPathCommand;
import frc.robot.subsystems.drive.Drive;

public class TestAuto implements AutoRoutine {
    private final Drive drive;
    private final PathPlannerPath test;

    public TestAuto(Drive drive) {
        this.drive = drive;
        test = PathPlannerUtil.loadTrajectory("test");
    }

    @Override
    public Command getCommand() {
        return new FollowPathCommand(drive, test);
    }

    @Override
    public Pose2d[] getPoses() {
        return test.getPathPoses().toArray(Pose2d[]::new);
    }
}
