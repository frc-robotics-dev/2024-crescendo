package frc.robot;

import frc.lib.math.AllianceFlipUtil;
import frc.robot.Constants.Mode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.WarmupExecutor;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOBasicSim;
import frc.robot.subsystems.drive.io.DriveIOMapleSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.VisionEstimateConsumer;
import frc.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import frc.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonSim;
import frc.robot.viz.GameViz;
import frc.robot.viz.PracticeMatchViz;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Main container for robot subsystems, commands, and controller bindings.
 * Use https://www.padcrafter.com to visualize the controller bindings.
 */
public class RobotContainer {
    // Subsystems
    private Drive drive;
    private Vision vision;

    // Sim
    private final GameViz gameViz;
    private final VisionSystemSim visionViz = new VisionSystemSim("AprilTagSimulator");

    // Auto
    private final AutoSelector autoChooser;
    private final WarmupExecutor warmupExecutor;

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);

    // Main Buttons

    // Overrides
    private final Trigger toggleSlowMode = driverXbox.back();
    private final Trigger toggleRobotRelative = driverXbox.start();
    private final Trigger resetRobotRotation = driverXbox.povUp();
    private final Trigger xWheels = driverXbox.povDown();

    // Commands
    private final TeleopDriveCommand teleopDriveCommand;
    
    // Other
    private final VisionEstimateConsumer visionEstimateConsumer =
        (pose, timestamp, stdDevs) -> drive.addVisionMeasurement(pose, timestamp, stdDevs);

    public RobotContainer() {
        // Initialize subsystems based on robot type
        if (Constants.getMode() != Mode.REPLAY) {
            switch (Constants.getRobot()) {
                case CompBot -> {
                    // Not implemented
                }
                case PracticeBot -> {
                    // Not implemented
                }
                case ProgrammingBot -> {
                    // Not implemented
                }
                case SimBot -> {
                    drive =
                        new Drive(
                            Constants.usingMapleSim
                                ? new DriveIOMapleSim()
                                : new DriveIOBasicSim());

                    visionViz.addAprilTags(FieldConstants.aprilTagFieldLayout);

                    vision =
                        new Vision(
                            visionEstimateConsumer,
                            new AprilTagIO[] {
                                new AprilTagIOPhotonSim(VisionConstants.leftCameraConfig, visionViz),
                                new AprilTagIOPhotonSim(VisionConstants.rightCameraConfig, visionViz),
                                new AprilTagIOPhotonSim(VisionConstants.backCameraConfig, visionViz),
                                new AprilTagIOPhotonSim(VisionConstants.backLeftCameraConfig, visionViz)
                            }
                        );
                }
            }
        }

        // No-op implementations if replay or not defined above
        if (drive == null) {
            drive = new Drive(new DriveIO() {});
        }

        if (vision == null) {
            vision = new Vision(visionEstimateConsumer, new AprilTagIO[] {});
        }

        // Create sim requirements
        gameViz =
            Constants.isPracticeMatch
                ? new PracticeMatchViz(drive, visionViz)
                : new GameViz(drive, visionViz);

        // Create auto requirements
        autoChooser = new AutoSelector(drive, gameViz);
        warmupExecutor = new WarmupExecutor(drive);

        // Initialize commands
        teleopDriveCommand = new TeleopDriveCommand(drive, driverXbox);

        // Configure default commands
        drive.setDefaultCommand(teleopDriveCommand);

        // Configure button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Bind main controls

        // Bind override controls
        toggleSlowMode
            .onTrue(
                Commands.runOnce(teleopDriveCommand::toggleSlowMode)
                    .withName("Toggling Slow Mode"));

        toggleRobotRelative
            .onTrue(
                Commands.runOnce(teleopDriveCommand::toggleRobotRelative)
                    .withName("Toggling Robot Relative Mode"));

        resetRobotRotation
            .onTrue(
                Commands.runOnce(() -> drive.setAngle(AllianceFlipUtil.apply(Rotation2d.kZero)))
                    .withName("Reset Robot Rotation"));

        xWheels
            .onTrue(
                Commands.runOnce(drive::stopWithX)
                    .withName("Stop With X"));
    }

    public void robotPeriodic() {
        
    }

    public void autonomousInit() {        
        autoChooser.startAuto();
    }

    public void teleopInit() {
        autoChooser.cancelAuto();
    }

    public void disabledInit() {
        warmupExecutor.initialize();
    }

    public void disabledPeriodic() {
        autoChooser.updateAutoSelection();
        warmupExecutor.update();
    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void simulationInit() {
        if (Constants.usingMapleSim) {
            for (int i = 0; i < 2; i++) { // Do twice to counteract MapleSim arena initialization effects
                drive.setPose(AllianceFlipUtil.apply(new Pose2d(1.889, 4.002, Rotation2d.kZero)));
            }
        }
    }

    public void simulationPeriodic() {
        gameViz.update();
    }
}