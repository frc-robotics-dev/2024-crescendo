package frc.robot.commands.tuning;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TestSelector {
    private final LoggedDashboardChooser<Command> selector;

    public TestSelector() {
        selector = new LoggedDashboardChooser<>("Test");
    }

    public void initialize() {
        CommandScheduler.getInstance().schedule(selector.get());
    }
}