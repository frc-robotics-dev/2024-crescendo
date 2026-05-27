package frc.robot.commands;

import frc.lib.rebuilt.ClimbUtil;
import frc.robot.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AlignToClimb extends SequentialCommandGroup {
    public AlignToClimb(Drive drive) {
        addCommands(
            new DriveToPose(drive, () -> ClimbUtil.getPreClimbPose(drive.getPose())),
            new DriveToPose(drive, () -> ClimbUtil.getClimbPose(drive.getPose())));
    }
}