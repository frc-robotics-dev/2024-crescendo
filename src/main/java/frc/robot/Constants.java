package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** Class containing global configuration variables describing current robot and runtime mode. */
public class Constants {
    private static RobotType robot = RobotType.SimBot;

    public static final boolean usingMapleSim = true;
    public static final boolean isPracticeMatch = true;

    public static final double loopPeriodSecs = LoggedRobot.defaultPeriodSecs;

    public static RobotType getRobot() {
        if (RobotBase.isReal() && robot == RobotType.SimBot) {
            DriverStation.reportError("Invalid robot selected, using competition robot as default.", false);
            robot = RobotType.CompBot;
        }
        return robot;
    }

    public static Mode getMode() {
        return switch (robot) {
            case CompBot, PracticeBot, ProgrammingBot -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SimBot -> Mode.SIM;
        };
    }

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        CompBot, PracticeBot, ProgrammingBot, SimBot
    }
}