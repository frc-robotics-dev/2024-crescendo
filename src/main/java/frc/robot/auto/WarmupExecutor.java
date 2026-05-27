package frc.robot.auto;

import java.io.File;

import frc.lib.auto.pathplanner.PathPlannerUtil;
import frc.robot.subsystems.drive.Drive;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public final class WarmupExecutor {
    private final Drive drive;

    private final String pathPlannerPathSuffix = ".path";

    private final int maxWarmupCycles = 50; // ~1 sec at 20ms loop
    private int warmupCycles = 0;

    public WarmupExecutor(Drive drive) {
        this.drive = drive;
    }

    public void initialize() {
        // Warmup PathPlanner cmds
        CommandScheduler.getInstance().schedule(
            FollowPathCommand
                .warmupCommand()
                .withName("FollowPathCommand Warmup")
                .ignoringDisable(true),
                
            PathfindingCommand
                .warmupCommand()
                .withName("PathfindingCommand Warmup")
                .ignoringDisable(true));

        // Load paths
        printWarmupTime("PathPlanner warmup", this::warmupPathPlannerPaths);
    }

    public void update() {
        if (warmupCycles < maxWarmupCycles) {
            warmupShotCalculator();
            warmupCycles++;
        }
    }

    private void warmupShotCalculator() {
        
    }

    private void warmupPathPlannerPaths() {
        String pathPlannerDir = Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner/paths";
        File[] files = new File(pathPlannerDir).listFiles((dir, name) -> name.endsWith(pathPlannerPathSuffix));

        if (files == null) {
            System.err.println("Failed to warmup PathPlanner paths");
            return;
        }

        for (File file : files) {
            PathPlannerUtil.loadTrajectory(stripExtension(file.getName(), pathPlannerPathSuffix));
        }
    }
    
    // Utility methods
    private void printWarmupTime(String key, Runnable action) { // Wrap over another method to find warmup time
        long startTime = System.nanoTime();
        action.run();
        long endTime = System.nanoTime();
        System.out.println(key + " took " + (endTime - startTime) / 1e9 + " seconds.");
    }

    private String stripExtension(String fileName, String extension) {
        return fileName.substring(0, fileName.length() - extension.length());
    }
}