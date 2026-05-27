package frc.robot.subsystems.vision.io.apriltagdetection;

import org.photonvision.simulation.VisionSystemSim;

import frc.robot.subsystems.vision.VisionConstants.CameraConfig;

public class AprilTagIOPhotonSim extends AprilTagIOPhotonVision {
    private final VisionSystemSim viz;

    public AprilTagIOPhotonSim(CameraConfig config, VisionSystemSim viz) {
        super(config);
        this.viz = viz;
    }
}