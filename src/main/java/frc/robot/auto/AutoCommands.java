package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class AutoCommands {
    private final Drive drive;

    public AutoCommands(Drive drive) {
        this.drive = drive;
    }

    public Command coastDrive() {
        return
            Commands.runOnce(drive::coast)
                .ignoringDisable(true)
                .withName("Coast Drive");
    }
}