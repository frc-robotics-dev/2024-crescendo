package frc.robot.auto;

import frc.lib.auto.pathplanner.PathPlannerUtil;
import frc.robot.auto.autos.ScorePreloadAndDriveStraight;
import frc.robot.subsystems.drive.Drive;
import frc.robot.viz.GameViz;
import frc.robot.viz.SimConstants;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class AutoSelector extends LoggedDashboardChooser<AutoRoutine> {
    private final GameViz gameViz;

    private Command autoCommand;

    public AutoSelector(Drive drive, GameViz gameViz) {
        super("Auto");

        this.gameViz = gameViz;

        // Configure PathPlanner
        PathPlannerUtil.configureAutoBuilder(drive);
        Pathfinding.ensureInitialized();

        // Update trajectory when selection changes
        onChange(selected -> {
            if (selected == null) {
                logTrajectory();
                return;
            }

            Pose2d[] trajectoryPoses = selected.getPoses();

            drive.setPose(trajectoryPoses[0]); // Set drive pose to trajectory start
            logTrajectory(trajectoryPoses);
        });

        // Configure autos
        AutoCommands autoCommands = new AutoCommands(drive);

        addDefaultOption(
            "Score Preload, Drive Straight",
            new ScorePreloadAndDriveStraight(drive, autoCommands));
    }

    public void startAuto() {
        AutoRoutine selectedAuto = get();

        if (selectedAuto == null) {
            return;
        }

        autoCommand =
            selectedAuto
                .getCommand()
                .withName(selectedAuto.getClass().getSimpleName());

        if (RobotBase.isSimulation()) {
            autoCommand = autoCommand.withTimeout(SimConstants.autoTimeSec);
        }

        CommandScheduler.getInstance().schedule(autoCommand);
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