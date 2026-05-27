package frc.robot.auto;

import frc.lib.auto.pathplanner.LocalADStarAK;
import frc.lib.auto.pathplanner.PathPlannerUtil;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakeroller.IntakeRoller;
import frc.robot.subsystems.launcher.flywheels.Flywheels;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.viz.GameViz;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoChooser {
    private final Drive drive;
    private final GameViz gameViz;

    private final LoggedDashboardChooser<AutoMode> routineChooser = new LoggedDashboardChooser<>("Auto");

    private Command autoCommand;
    private AutoMode lastSelectedAuto;

    private final double simAutoTimeSec = 20;

    public AutoChooser(
        Drive drive,
        IntakePivot intakePivot,
        IntakeRoller intakeRoller,
        Indexer indexer,
        Feeder feeder,
        Hood hood,
        Flywheels flywheels,
        Climber climber,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.gameViz = gameViz;

        // Configure PathPlanner
        PathPlannerUtil.configureAutoBuilder(drive);
        Pathfinding.setPathfinder(new LocalADStarAK());

        // Configure autos
    }

    public void startAuto() {
        AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            return;
        }

        autoCommand =
            selectedAuto
                .getCommand()
                .withName(selectedAuto.getClass().getSimpleName());

        if (RobotBase.isSimulation()) {
            autoCommand = autoCommand.withTimeout(simAutoTimeSec);
        }

        CommandScheduler.getInstance().schedule(autoCommand);
    }

    public void updateAutoSelection() {
        final AutoMode selectedAuto = routineChooser.get();

        if (selectedAuto == null) {
            logTrajectory(); // Clear poses

        } else if (selectedAuto != lastSelectedAuto) {
            Pose2d[] trajectoryPoses = selectedAuto.getPoses();

            drive.setPose(trajectoryPoses[0]); // Set drive pose to trajectory start
            logTrajectory(trajectoryPoses);
        }

        lastSelectedAuto = selectedAuto;
    }

    public void cancelAuto() {
        logTrajectory(); // Clear poses

        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    private void logTrajectory(Pose2d... trajectory) {
        gameViz.getField2d().getObject("Trajectory").setPoses(trajectory);
        Logger.recordOutput("Drive/Trajectory", trajectory);
    }
}