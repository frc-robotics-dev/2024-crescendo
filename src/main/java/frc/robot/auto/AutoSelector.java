package frc.robot.auto;

import frc.lib.auto.pathplanner.PathPlannerUtil;
import frc.robot.auto.autos.TestAuto;
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

public class AutoSelector {
    private final Drive drive;
    private final GameViz gameViz;

    private final LoggedDashboardChooser<AutoRoutine> routineChooser = new LoggedDashboardChooser<>("Auto");

    private Command autoCommand;
    private AutoRoutine lastSelectedAuto;

    public AutoSelector(
        Drive drive,
        GameViz gameViz
    ) {
        this.drive = drive;
        this.gameViz = gameViz;

        // Configure PathPlanner
        PathPlannerUtil.configureAutoBuilder(drive);
        Pathfinding.ensureInitialized();

        // Configure autos
        AutoCommands autoCommands = new AutoCommands(drive);

        routineChooser.addDefaultOption("Test Auto", new TestAuto(drive));
    }

    public void startAuto() {
        AutoRoutine selectedAuto = routineChooser.get();

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

    public void updateAutoSelection() {
        final AutoRoutine selectedAuto = routineChooser.get();

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