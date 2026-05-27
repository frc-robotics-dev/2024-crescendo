package frc.robot.viz;

import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.IntakeSimulation;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class GameViz {
    private final Drive drive;

    private final VisionSystemSim visionViz;

    private IntakeSimulation intakeSimulation;

    public GameViz(
        Drive drive,
        VisionSystemSim visionViz
    ) {
        this.drive = drive;
        this.visionViz = visionViz;
    }

    public void update() {
        // Update visualizers
        visionViz.update(drive.getPose());
    }

    public Field2d getField2d() {
        return visionViz.getDebugField();
    }
}