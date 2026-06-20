package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.WarmupExecutor;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.tuning.SysIdCommand;
import frc.robot.commands.tuning.WheelRadiusCharacterization6328;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVision.VisionEstimateConsumer;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants;
import frc.robot.subsystems.apriltagvision.io.AprilTagIOPhotonSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOMapleSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.viz.GameViz;
import frc.robot.viz.PracticeMatchViz;

/**
 * Main container for robot subsystems, commands, and controller bindings.
 * Use https://www.padcrafter.com to visualize the controller bindings.
 */
public class RobotContainer {
    // Subsystems
    private Drive drive;
    private AprilTagVision aprilTagVision;

    // Path Builder
    private final FollowPath.Builder pathBuilder;

    // Sim
    private final GameViz gameViz;
    private final VisionSystemSim visionViz = new VisionSystemSim("AprilTagSimulator");

    // Auto
    private final AutoSelector autoSelector;
    private final WarmupExecutor warmupExecutor;

    // Test
    private final LoggedDashboardChooser<Command> testSelector = new LoggedDashboardChooser<>("Test");
    private Command testCommand;

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController operatorXbox = new CommandXboxController(1);

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
            switch (Constants.robot) {
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
                    if (Constants.usingMapleSim) {
                        drive = new Drive(new DriveIOMapleSim());
                    }

                    visionViz.addAprilTags(FieldConstants.aprilTagFieldLayout);

                    aprilTagVision =
                        new AprilTagVision(
                            visionEstimateConsumer,
                            new AprilTagIOPhotonSim(AprilTagVisionConstants.leftCameraConfig, visionViz),
                            new AprilTagIOPhotonSim(AprilTagVisionConstants.rightCameraConfig, visionViz),
                            new AprilTagIOPhotonSim(AprilTagVisionConstants.backCameraConfig, visionViz),
                            new AprilTagIOPhotonSim(AprilTagVisionConstants.backLeftCameraConfig, visionViz)
                        );
                }
            }
        }

        // No-op implementations if replay or not defined above
        if (drive == null) {
            drive = new Drive(new DriveIO() {});
        }

        if (aprilTagVision == null) {
            aprilTagVision = new AprilTagVision(visionEstimateConsumer);
        }

        // Create path builder
        pathBuilder =
            new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getRobotVelocity,
                drive::runVelocity,
                DriveConstants.linearPID.toPIDController(),
                DriveConstants.thetaPID.toPIDController(),
                DriveConstants.ctePID.toPIDController()
            )
            .withDefaultShouldFlip();

        // Create sim requirements
        gameViz =
            Constants.isPracticeMatch
                ? new PracticeMatchViz(drive, visionViz)
                : new GameViz(drive, visionViz);

        // Create auto requirements
        autoSelector = new AutoSelector(drive, gameViz);
        warmupExecutor = new WarmupExecutor();

        // Initialize commands
        teleopDriveCommand = new TeleopDriveCommand(drive, driverXbox);

        // Configure default commands, autos, button binds, tests
        configureDefaultCommands();

        configureAutos();
        configureMainButtonBindings();
        configureOverrideButtonBindings();

        configureTests();
    }

    private void configureDefaultCommands() {
        drive.setDefaultCommand(teleopDriveCommand);
    }

    private void configureAutos() {
        
    }

    private void configureMainButtonBindings() {
        
    }

    private void configureOverrideButtonBindings() {
        // Bind driver override controls
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
                Commands.runOnce(() -> drive.resetAngle(AllianceFlipUtil.apply(Rotation2d.kZero)))
                    .withName("Resetting Robot Rotation"));

        xWheels
            .onTrue(
                Commands.runOnce(drive::stopWithX)
                    .withName("Stopping With X"));

        // Bind operator override controls
    }

    private void configureTests() {
        testSelector.addDefaultOption(
            "Wheel Radius Characterization 6328",
            new WheelRadiusCharacterization6328(drive));

        testSelector.addOption(
            "Drive SysId",
            new SysIdCommand(drive, volts -> drive.runCharacterization(volts.in(Volts))));
    }

    public void autonomousInit() {
        autoSelector.startAuto();
    }

    public void autonomousExit() {
        autoSelector.stopAuto();
    }

    public void disabledInit() {
        warmupExecutor.initialize();
    }

    public void disabledPeriodic() {
        warmupExecutor.update();
    }

    public void testInit() {
        testCommand = testSelector.get();
        
        if (testCommand != null) {
            testCommand.schedule();
        }
    }

    public void testExit() {
        if (testCommand != null) {
            testCommand.cancel();
        }
    }

    public void simulationInit() {
        if (Constants.usingMapleSim) {
            drive.resetPose(AllianceFlipUtil.apply(new Pose2d(1.889, 4.002, Rotation2d.kZero)));
        }
    }

    public void simulationPeriodic() {
        gameViz.update();
    }
}