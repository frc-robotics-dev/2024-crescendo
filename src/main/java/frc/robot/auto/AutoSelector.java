package frc.robot.auto;

import frc.robot.lib.BLine.BLineField;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.viz.GameViz;
import frc.robot.viz.SimConstants;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector extends LoggedDashboardChooser<AutoRoutine> {
    private final Drive drive;
    private final GameViz gameViz;

    private Command autoCommand;

    public AutoSelector(Drive drive, GameViz gameViz) {
        super("Auto");
        
        this.drive = drive;
        this.gameViz = gameViz;

        onChange(selected -> {
            // Update trajectory
            Path[] trajectory = selected.getTrajectory();

            if (trajectory == null || trajectory.length == 0) { // If trajectory empty
                clearTrajectory();
                return;
            }

            for (Path path : trajectory) {
                BLineField.drawPath(gameViz.getField2d(), path);
            }

            // Reset pose
            drive.resetPose(selected.getStartPose());

            // Initialize command
            autoCommand = selected;

            if (RobotBase.isSimulation()) {
                autoCommand = autoCommand.withTimeout(SimConstants.autoTimeSec);
            }

            autoCommand.setName(selected.getName());
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
        BLineField.drawPath(gameViz.getField2d(), new Path(new Waypoint(drive.getPose())));
    }
}