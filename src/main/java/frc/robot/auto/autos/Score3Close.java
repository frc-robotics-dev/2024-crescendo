package frc.robot.auto.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoRoutine;

public class Score3Close implements AutoRoutine {
    public Score3Close() {}
    
    @Override
    public Command getCommand() {
        return Commands.none();
    }

    @Override
    public Pose2d[] getPoses() {
        return new Pose2d[0];
    }
}