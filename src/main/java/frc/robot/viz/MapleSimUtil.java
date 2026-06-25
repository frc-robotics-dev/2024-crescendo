package frc.robot.viz;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.Arena2024Crescendo;

public class MapleSimUtil {
    private MapleSimUtil() {}
    
    public static void initializeArena() {
        SimulatedArena.overrideInstance(new Arena2024Crescendo());
    }
}