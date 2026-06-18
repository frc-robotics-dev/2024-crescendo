package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class CoastDrive extends Command {
    private final Drive drive;

    public CoastDrive(Drive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drive.coast();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}