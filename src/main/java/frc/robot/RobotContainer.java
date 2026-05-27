package frc.robot;

import frc.lib.math.AllianceFlipUtil;
import frc.lib.math.MathUtils;
import frc.robot.Constants.Mode;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.WarmupExecutor;
import frc.robot.commands.AimAndPrepShot;
import frc.robot.commands.AlignToClimb;
import frc.robot.commands.EjectFuelFromIntake;
import frc.robot.commands.IntakeFuelFromGround;
import frc.robot.commands.LowerClimber;
import frc.robot.commands.RaiseClimber;
import frc.robot.commands.PutIntakeUpForShoot;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.io.ClimberIO;
import frc.robot.subsystems.climber.io.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.io.DriveIO;
import frc.robot.subsystems.drive.io.DriveIOBasicSim;
import frc.robot.subsystems.drive.io.DriveIOMapleSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.feeder.io.FeederIO;
import frc.robot.subsystems.feeder.io.FeederIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.io.IndexerIO;
import frc.robot.subsystems.indexer.io.IndexerIOSim;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakepivot.io.IntakePivotIO;
import frc.robot.subsystems.intakepivot.io.IntakePivotIOSim;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.intakeroller.io.IntakeRollerIO;
import frc.robot.subsystems.intakeroller.io.IntakeRollerIOSim;
import frc.robot.subsystems.launcher.LaunchCalculator;
import frc.robot.subsystems.launcher.LaunchPreset;
import frc.robot.subsystems.launcher.LaunchCalculator.ShotInfo;
import frc.robot.subsystems.launcher.flywheels.Flywheels;
import frc.robot.subsystems.launcher.flywheels.FlywheelsConstants;
import frc.robot.subsystems.launcher.flywheels.io.FlywheelsIO;
import frc.robot.subsystems.launcher.flywheels.io.FlywheelsIOSim;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.hood.HoodConstants;
import frc.robot.subsystems.launcher.hood.io.HoodIO;
import frc.robot.subsystems.launcher.hood.io.HoodIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.VisionEstimateConsumer;
import frc.robot.subsystems.vision.io.apriltagdetection.AprilTagIO;
import frc.robot.subsystems.vision.io.apriltagdetection.AprilTagIOPhotonSim;
import frc.robot.viz.GameViz;
import frc.robot.viz.PracticeMatchViz;
import frc.robot.viz.VisionSimulator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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
    private IntakePivot intakePivot;
    private IntakeRoller intakeRoller;
    private Indexer indexer;
    private Feeder feeder;
    private Hood hood;
    private Flywheels flywheels;
    private Climber climber;

    // Sim
    private final GameViz gameViz;
    private final VisionSimulator visionViz = new VisionSimulator();

    // Auto
    private final AutoChooser autoChooser;
    private final WarmupExecutor warmupExecutor;

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);

    // Main Buttons
    private final Trigger intakeGround = driverXbox.leftTrigger();
    private final Trigger ejectIntake = driverXbox.leftBumper();
    
    private final Trigger shootFuel = driverXbox.rightTrigger();
    private final Trigger aimAndPrepShot = driverXbox.rightBumper();

    private final Trigger setBatterPreset = driverXbox.y();
    private final Trigger setTrenchPreset = driverXbox.x();
    private final Trigger setDepotPreset = driverXbox.a();

    private final Trigger climb = driverXbox.b();

    // Overrides
    private final Trigger toggleSlowMode = driverXbox.back();
    private final Trigger toggleRobotRelative = driverXbox.start();
    private final Trigger resetRobotRotation = driverXbox.povUp();
    private final Trigger xWheels = driverXbox.povDown();
    private final Trigger alignToClimb = driverXbox.povRight();

    // Commands
    private final TeleopDriveCommand teleopDriveCommand;
    
    // Other
    private final VisionEstimateConsumer visionEstimateConsumer = drive::addVisionMeasurement;

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

                    vision =
                        new Vision(
                            visionEstimateConsumer,
                            new AprilTagIO[] {
                                new AprilTagIOPhotonSim(VisionConstants.leftCameraConfig),
                                new AprilTagIOPhotonSim(VisionConstants.rightCameraConfig),
                                new AprilTagIOPhotonSim(VisionConstants.backCameraConfig),
                                new AprilTagIOPhotonSim(VisionConstants.backLeftCameraConfig)
                            }
                        );

                    intakePivot = new IntakePivot(new IntakePivotIOSim());
                    intakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                    indexer = new Indexer(new IndexerIOSim());
                    feeder = new Feeder(new FeederIOSim());
                    hood = new Hood(new HoodIOSim());
                    flywheels = new Flywheels(new FlywheelsIOSim());

                    climber = new Climber(new ClimberIOSim());
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

        if (intakePivot == null) {
            intakePivot = new IntakePivot(new IntakePivotIO() {});
        }
        
        if (intakeRoller == null) {
            intakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        }

        if (indexer == null) {
            indexer = new Indexer(new IndexerIO() {});
        }

        if (feeder == null) {
            feeder = new Feeder(new FeederIO() {});
        }
            
        if (hood == null) {
            hood = new Hood(new HoodIO() {});
        }

        if (flywheels == null) {
            flywheels = new Flywheels(new FlywheelsIO() {});
        }

        if (climber == null) {
            climber = new Climber(new ClimberIO() {});
        }

        // Create sim requirements
        gameViz =
            Constants.isPracticeMatch
                ? new PracticeMatchViz(drive, intakePivot, intakeRoller, hood, flywheels, climber, visionViz)
                : new GameViz(drive, intakePivot, intakeRoller, hood, flywheels, climber, visionViz);
        
        drive.setTrenchCollisionSim(gameViz.getTrenchCollisionSim());

        // Create auto requirements
        autoChooser = new AutoChooser(drive, intakePivot, intakeRoller, indexer, feeder, hood, flywheels, climber, gameViz);
        warmupExecutor = new WarmupExecutor(drive);

        // Initialize commands
        teleopDriveCommand = new TeleopDriveCommand(drive, driverXbox);

        // Configure default commands
        drive.setDefaultCommand(teleopDriveCommand);
        
        feeder.setDefaultCommand(
            Commands.runOnce((() -> feeder.setVelocity(FeederConstants.IDLE)), feeder)
                .withName("Feeder Default Command"));

        hood.setDefaultCommand(
            Commands.runOnce(() -> hood.setAngle(HoodConstants.DUCK_UNDER_TRENCH, 0.0), hood)
                .withName("Hood Default Command"));

        // Configure button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Bind main controls
        intakeGround
            .whileTrue(new IntakeFuelFromGround(intakePivot, intakeRoller, gameViz));

        shootFuel
            .whileTrue(new ShootFuel(indexer, gameViz))
            .and(intakeGround.negate())
            .whileTrue(new PutIntakeUpForShoot(intakePivot, intakeRoller, gameViz));

        aimAndPrepShot
            .whileTrue(new AimAndPrepShot(drive, feeder, hood, flywheels, driverXbox));

        ejectIntake
            .whileTrue(new EjectFuelFromIntake(intakePivot, intakeRoller, indexer, feeder));

        bindShotPreset(setBatterPreset, LaunchPreset.BATTER);
        bindShotPreset(setTrenchPreset, LaunchPreset.TRENCH);
        bindShotPreset(setDepotPreset, LaunchPreset.TOWER);

        climb
            .onTrue(new RaiseClimber(climber))
            .onFalse(new LowerClimber(climber, gameViz));

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

        alignToClimb
            .whileTrue(new AlignToClimb(drive));
    }

    private void bindShotPreset(Trigger trigger, LaunchPreset shotPreset) {
        trigger
            .onTrue(Commands.runOnce(() -> LaunchCalculator.getInstance().setShotPreset(shotPreset)))
            .onFalse(Commands.runOnce(() -> LaunchCalculator.getInstance().setShotPreset(LaunchPreset.NONE)));
    }

    public void robotPeriodic() {
        // Calculate latest shot info
        ShotInfo shotInfo =
            LaunchCalculator.getInstance().calculateShotInfo(
                drive.getPose(),
                drive.getRobotVelocity(),
                drive.getFieldVelocity());

        // Check if shot feasible
        boolean shotDistanceValid =
            LaunchCalculator.inAllianceZone(drive.getPose())
                ? MathUtils.inRange(shotInfo.launcherToTargetDistance(), LaunchCalculator.minDistanceHubShoot, LaunchCalculator.maxDistanceHubShoot)
                : MathUtils.inRange(shotInfo.launcherToTargetDistance(), LaunchCalculator.minDistanceLobShoot, LaunchCalculator.maxDistanceLobShoot);
                
        boolean driveAtGoal = MathUtil.isNear(shotInfo.driveAngle().getRadians(), drive.getRotation().getRadians(), DriveConstants.aimTolerance);
        boolean hoodAtGoal = hood.isAtAngle(shotInfo.hoodAngleRad(), HoodConstants.shootOnMoveTolerance);
        boolean flywheelsAtGoal = flywheels.isAtVelocity(shotInfo.flywheelsVelocityRadPerSec(), FlywheelsConstants.tolerance);

        boolean isCalculatedShotFeasible =
            shotDistanceValid && driveAtGoal && hoodAtGoal && flywheelsAtGoal;

        LaunchCalculator.getInstance().setShotFeasible(
            isCalculatedShotFeasible ||
            LaunchCalculator.getInstance().getShotPreset() != LaunchPreset.NONE); // shot feasible is true (if using preset)

        // Log data
        Logger.recordOutput("ShotCalculator/Shot Distance Valid?", shotDistanceValid);
        Logger.recordOutput("ShotCalculator/Drive At Goal?", driveAtGoal);
        Logger.recordOutput("ShotCalculator/Hood At Goal?", hoodAtGoal);
        Logger.recordOutput("ShotCalculator/Flywheels At Goal?", flywheelsAtGoal);

        // Clear latest shot info
        LaunchCalculator.getInstance().clearLatestShotInfo();
    }

    public void autonomousInit() {        
        autoChooser.startAuto();
    }

    public void teleopInit() {
        autoChooser.cancelAuto();
    }

    public void disabledInit() {
        if (drive.shouldCoastAfterAutoEnd()) {
            drive.coast(); // Coasts drivetrain in disabled mode if post-auto coasting is enabled
        }

        warmupExecutor.initialize();
    }

    public void disabledPeriodic() {
        autoChooser.updateAutoSelection();
        warmupExecutor.update();
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

    public void test() {
        
    }
}