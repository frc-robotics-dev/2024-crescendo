package frc.robot.auto;

import frc.robot.commands.AimAndPrepShot;
import frc.robot.commands.AlignToClimb;
import frc.robot.commands.IntakeFuelFromGround;
import frc.robot.commands.LowerClimber;
import frc.robot.commands.PutIntakeUpForShoot;
import frc.robot.commands.RaiseClimber;
import frc.robot.commands.ShootFuel;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.launcher.flywheels.Flywheels;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.viz.GameViz;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public abstract class AutoMode {
    private final Drive drive;
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Hood hood;
    private final Flywheels flywheels;
    private final Climber climber;
    private final GameViz gameViz;

    public AutoMode(
        Drive drive,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Indexer indexer,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
        this.indexer = indexer;
        this.feeder = feeder;
        this.hood = hood;
        this.flywheels = flywheels;
        this.climber = climber;
        this.gameViz = gameViz;
    }

    protected abstract Command getCommand();
    protected abstract Pose2d[] getPoses();

    protected Command intake() {
        return new IntakeFuelFromGround(intakePivot, intakeRoller, gameViz);
    }

    protected Command shoot() {
        return
            Commands.parallel(
                new AimAndPrepShot(drive, feeder, hood, flywheels),
                new ShootFuel(indexer, gameViz),
                new PutIntakeUpForShoot(intakePivot, intakeRoller, gameViz));
    }

    protected Command autoClimb() {
        return
            Commands.sequence(
                new RaiseClimber(climber),
                new AlignToClimb(drive),
                new LowerClimber(climber, gameViz));
    }
}