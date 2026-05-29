package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoCommands;
import frc.robot.auto.AutoRoutine;
import frc.robot.subsystems.drive.Drive;

public class ScorePreloadAndDriveStraight implements AutoRoutine {
    private final Drive drive;
    private final AutoCommands autoCommands;

    private final double driveBackDist = Units.feetToMeters(1);

    public ScorePreloadAndDriveStraight(Drive drive, AutoCommands autoCommands) {
        this.drive = drive;
        this.autoCommands = autoCommands;
    }

    @Override
    public Command getCommand() {
        return Commands.sequence(
            autoCommands.shootNote().withTimeout(3),
            driveBack()
        );
    }

    @Override
    public Pose2d[] getPoses() {
        return new Pose2d[0];
    }

    private Command driveBack() {
        double driveTime = 1.0;
        double driveVelocity = driveBackDist / driveTime;

        return
            Commands
                .run(() -> drive.runVelocity(new ChassisSpeeds(driveVelocity, 0, 0)))
                .withTimeout(driveTime);
    }
}
