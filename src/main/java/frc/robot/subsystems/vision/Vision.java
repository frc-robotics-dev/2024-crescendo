package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.VisionEstimateConsumer;
import frc.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import frc.robot.subsystems.vision.io.apriltagdetection.AprilTagIOInputsAutoLogged;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final AprilTagIO[] io;
    private final AprilTagIOInputsAutoLogged[] inputs;

    private final VisionEstimateConsumer visionEstimateConsumer;
    
    public Vision(VisionEstimateConsumer visionEstimateConsumer, AprilTagIO[] io) {
        this.visionEstimateConsumer = visionEstimateConsumer;

        this.io = io;
        this.inputs = new AprilTagIOInputsAutoLogged[io.length];
        
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new AprilTagIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        
    }
}