package frc.lib.subsystem;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggerUtil;

/** Wrapper class around WPILib's {@link SubsystemBase} class to create command-based subsystems with common boilerplate methods. */
public abstract class Mechanism extends SubsystemBase {
    protected LoggedNetworkBoolean coastOverride = new LoggedNetworkBoolean("Coast Mode/" + getName(), false);
    private boolean inCoast = false;

    @Override
    public void periodic() {
        LoggerUtil.recordCurrentCommand(this);

        // Set coast mode only when disabled and there is a change in the override
        boolean shouldCoast = coastOverride.get();

        if (DriverStation.isDisabled() && shouldCoast != inCoast) {
            setBrakeMode(!shouldCoast);
            inCoast = shouldCoast;
        }

        if (DriverStation.isEnabled() && coastOverride.get()) {
            DriverStation.reportWarning(getName() + " cannot coast while running! Request ignored.", false);
        }
    };

    protected abstract void setBrakeMode(boolean enabled);
    public abstract void stop();
}