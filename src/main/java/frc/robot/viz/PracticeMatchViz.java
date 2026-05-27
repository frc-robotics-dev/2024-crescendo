package frc.robot.viz;

import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class PracticeMatchViz extends GameViz {
    private int score = 0;
    
    public PracticeMatchViz(Drive drive, VisionSystemSim visionViz) {
        super(drive, visionViz);
    }

    @Override
    public void update() {
        super.update();
        
        // Log score
        Logger.recordOutput("PracticeMatchViz/Score", score);
    }

    private boolean isMatchEnded() {
        return false;
    }
}