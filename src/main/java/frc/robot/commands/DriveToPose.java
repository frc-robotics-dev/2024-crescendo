package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private final Drive drive;
    private final Pose2d targetPose;
    
    private final double xyTolerance;
    private final double thetaTolerance = Units.degreesToRadians(2.0);
    private final boolean isWaypoint;

    private final ProfiledPIDController xController =
        new ProfiledPIDController(
            4.0,
            0.0,
            0.0,
            new Constraints(3.8, 3.0));
        
    private final ProfiledPIDController yController =
        new ProfiledPIDController(
            4.0,
            0.0,
            0.0,
            new Constraints(3.8, 3.0));
        
    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            4.0,
            0.0,
            0.0,
            new Constraints(4.0, 3.0));

    public DriveToPose(Drive drive, Pose2d targetPose, double xyTolerance, boolean isWaypoint) {
        this.drive = drive;
        this.targetPose = targetPose;
        this.xyTolerance = xyTolerance;
        this.isWaypoint = isWaypoint;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(xyTolerance);
        yController.setTolerance(xyTolerance);
        thetaController.setTolerance(thetaTolerance);

        addRequirements(drive);
    }

    public DriveToPose(Drive drive, Pose2d targetPose) {
        this(drive, targetPose, Units.inchesToMeters(2), false);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();
        ChassisSpeeds currentVel = drive.getFieldVelocity();

        // Seed profiles with robot pose & velocity
        xController.reset(currentPose.getX(), currentVel.vxMetersPerSecond);
        yController.reset(currentPose.getY(), currentVel.vyMetersPerSecond);
        thetaController.reset(currentPose.getRotation().getRadians(), currentVel.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drive.getPose();

        // Calculate speeds
        double xSpeed =
            xController.calculate(currentPose.getX(), targetPose.getX());

        double ySpeed =
            yController.calculate(currentPose.getY(), targetPose.getY());

        double thetaSpeed =
            thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        // Prevent stutter if specific axis reached early
        if (!isWaypoint) {
            if (xController.atGoal()) xSpeed = 0.0;
            if (yController.atGoal()) ySpeed = 0.0;
            if (thetaController.atGoal()) thetaSpeed = 0.0;
        }

        // Command speeds
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                currentPose.getRotation()));

        // Log data
        Logger.recordOutput("DriveToPose/TargetPose", targetPose);
        Logger.recordOutput(
            "DriveToPose/SetpointPose", 
            new Pose2d(
                xController.getSetpoint().position,
                yController.getSetpoint().position,
                edu.wpi.first.math.geometry.Rotation2d.fromRadians(thetaController.getSetpoint().position)));
    }

    @Override
    public boolean isFinished() {
        if (isWaypoint) {
            return drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < xyTolerance;
        } else {
            return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        
        // Clear logs
        Logger.recordOutput("DriveToPose/TargetPose", Pose2d.kZero);
        Logger.recordOutput("DriveToPose/SetpointPose", Pose2d.kZero);
    }
}