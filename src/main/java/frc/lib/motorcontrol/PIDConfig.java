package frc.lib.motorcontrol;

import edu.wpi.first.math.controller.PIDController;

public record PIDConfig(
    double kP,
    double kI,
    double kD
) {
    public PIDConfig() {
        this(0.0, 0.0, 0.0);
    }

    public PIDController toPIDController() {
        return new PIDController(kP, kI, kD);
    }
}