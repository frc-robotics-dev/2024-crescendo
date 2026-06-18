package frc.robot.auto;

import frc.robot.lib.BLine.BLineField;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.drive.Drive;
import frc.robot.viz.GameViz;
import frc.robot.viz.SimConstants;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector extends LoggedDashboardChooser<AutoRoutine> {
    private final GameViz gameViz;

    private Command autoCommand;

    public AutoSelector(Drive drive, GameViz gameViz) {
        super("Auto");
        
        this.gameViz = gameViz;

        onChange(selected -> {
            // Reset pose & update trajectory
            Path[] trajectory = selected.getTrajectory();

            if (isTrajectoryEmpty(trajectory)) {
                clearTrajectory();
                return;
            }

            drive.resetPose(selected.getStartPose());
            
            for (Path path : trajectory) {
                BLineField.drawPath(gameViz.getField2d(), path);
            }

            // Initialize command
            autoCommand = selected.getCommand();

            if (RobotBase.isSimulation()) {
                autoCommand = autoCommand.withTimeout(SimConstants.autoTimeSec);
            }

            autoCommand.setName(selected.getClass().getSimpleName());
        });
    }

    public void startAuto() {
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    public void stopAuto() {
        clearTrajectory();

        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    private void clearTrajectory() {
        BLineField.drawPath(gameViz.getField2d(), new Path());
    }

    private boolean isTrajectoryEmpty(Path... trajectory) {
        return trajectory == null || trajectory.length == 0;
    }
}