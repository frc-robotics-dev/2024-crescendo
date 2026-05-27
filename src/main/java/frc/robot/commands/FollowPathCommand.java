package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.auto.GeometricFollower;
import frc.robot.subsystems.drive.Drive;

public class FollowPathCommand extends Command {
    private final Drive drive;
    private final GeometricFollower follower;

    public FollowPathCommand(Drive drive, PathPlannerPath ppPath) {
        this.drive = drive;

        List<PathPoint> pts = ppPath.getAllPathPoints();
        List<Translation2d> positions = new ArrayList<>();
        for (PathPoint pt : pts) {
            positions.add(pt.position);
        }

        List<RotationTarget> rotationTargets = ppPath.getRotationTargets();

        List<GeometricFollower.Sample> samples =
            GeometricFollower.buildSamplesFromPathPlanner(positions, rotationTargets);

        PIDController thetaPid = new PIDController(5.0, 0.0, 0.0);
        thetaPid.enableContinuousInput(-Math.PI, Math.PI);

        follower = new GeometricFollower(samples, thetaPid, 1.0, 2.0);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        double speed = 4.0;

        ChassisSpeeds speeds = follower.update(pose, speed);
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}