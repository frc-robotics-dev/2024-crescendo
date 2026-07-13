package frc.robot.subsystems.apriltagvision;

import frc.robot.subsystems.apriltagvision.io.AprilTagIO;
import frc.robot.subsystems.apriltagvision.io.AprilTagIOInputsAutoLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagVision extends SubsystemBase {
    private final AprilTagIO[] io;
    private final AprilTagIOInputsAutoLogged[] inputs;

    private final VisionEstimateConsumer visionEstimateConsumer;
    
    public AprilTagVision(VisionEstimateConsumer visionEstimateConsumer, AprilTagIO... io) {
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

    @FunctionalInterface
    public interface VisionEstimateConsumer {
        public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }
}