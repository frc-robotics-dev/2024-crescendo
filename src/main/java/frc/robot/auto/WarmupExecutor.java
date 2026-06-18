package frc.robot.auto;

public final class WarmupExecutor {
    private final int maxWarmupCycles = 10 * (1000 / 20); // 10 sec with 20ms loop
    private int warmupCycles = 0;

    public WarmupExecutor() {}

    public void initialize() {
        
    }

    public void update() {
        if (warmupCycles < maxWarmupCycles) {
            warmupCycles++;
        }
    }
    
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