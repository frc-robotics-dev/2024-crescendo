package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands {
    private final Drive drive;

    public AutoCommands(Drive drive) {
        this.drive = drive;
    }

    public Command driveToPose(Pose2d targetPose) {
        return new DriveToPose(drive, targetPose);
    }

    public Command driveToCenterlineNote(CenterlineNote note, Rotation2d intakeAngle) {
        return new DriveToPose(drive, new Pose2d(note.getTranslation(), intakeAngle));
    }

    public Command coastDrive() {
        return
            Commands.runOnce(drive::coast)
                .ignoringDisable(true)
                .withName("Coast Drive");
    }

    public Command shootNote() {
        return Commands.none();
    }
}