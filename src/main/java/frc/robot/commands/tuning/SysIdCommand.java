package frc.robot.commands.tuning;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdCommand extends SequentialCommandGroup {
    private final Consumer<Voltage> consumer;
    private final SysIdRoutine routine;

    public SysIdCommand(
        SubsystemBase subsystem,
        Consumer<Voltage> consumer,
        Velocity<VoltageUnit> rampRate,
        Voltage stepVoltage,
        Time timeout
    ) {
        // Create routine
        this.consumer = consumer;
        this.routine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    rampRate,
                    stepVoltage,
                    timeout,
                    state -> Logger.recordOutput(subsystem.getName() + "/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    consumer,
                    null, // No log consumer, since data recorded by AdvantageKit
                    subsystem));

        addCommands(
            sysIdQuasistatic(SysIdRoutine.Direction.kForward),
            sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
            sysIdDynamic(SysIdRoutine.Direction.kForward),
            sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /** Use this constructor if you want the default values (from SysIdRoutine.Config) for ramp rate, step voltage, and timeout. */
    public SysIdCommand(SubsystemBase subsystem, Consumer<Voltage> consumer) {
        this(subsystem, consumer, null, null, null);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return
            Commands.run(() -> consumer.accept(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(routine.quasistatic(direction));
    }
    
    /** Returns a command to run a dynamic test in the specified direction. */
    private Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return
            Commands.run(() -> consumer.accept(Volts.of(0.0)))
                .withTimeout(1.0)
                .andThen(routine.dynamic(direction));
    }
}
