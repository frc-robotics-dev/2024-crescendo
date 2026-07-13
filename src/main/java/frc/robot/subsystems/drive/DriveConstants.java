package frc.robot.subsystems.drive;

import frc.robot.Constants;
import frc.robot.subsystems.drive.generated.TunerConstantsCompBot;
import frc.robot.subsystems.drive.generated.TunerConstantsPracticeBot;
import frc.robot.subsystems.drive.generated.TunerConstantsProgrammingBot;
import frc.lib.motorcontrol.PIDConfig;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    // Hardware / Configuration
    public static final SwerveDrivetrainConstants drivetrainConstants;
    public static final SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    public static final double trackWidthX;
    public static final double trackWidthY;
    public static final double bumperLength = Units.inchesToMeters(30);
    public static final double bumperWidth = Units.inchesToMeters(35);

    public static final double driveBaseRadius;

    public static final double maxLinearSpeed;
    public static final double maxOmega;

    public static final double mass = Units.lbsToKilograms(87.4200862);
    public static final double wheelCOF = 1.9;

    public static final Translation2d centerOfRotation = Translation2d.kZero;

    // Swerve Control
    public static final PIDConfig linearPID = new PIDConfig(5, 0, 0);
    public static final PIDConfig thetaPID = new PIDConfig(3, 0, 0);
    public static final PIDConfig ctePID = new PIDConfig(2, 0, 0);

    static {
        // Get drivetrain & module constants from generated values (defaults to comp bot)
        switch (Constants.getRobot()) {
            case PracticeBot -> {
                drivetrainConstants = TunerConstantsPracticeBot.DrivetrainConstants;
                moduleConstants =
                    new SwerveModuleConstants<?, ?, ?>[] {
                        TunerConstantsPracticeBot.FrontLeft,
                        TunerConstantsPracticeBot.FrontRight,
                        TunerConstantsPracticeBot.BackLeft,
                        TunerConstantsPracticeBot.BackRight
                    };
            }
            case ProgrammingBot -> {
                drivetrainConstants = TunerConstantsProgrammingBot.DrivetrainConstants;
                moduleConstants =
                    new SwerveModuleConstants<?, ?, ?>[] {
                        TunerConstantsProgrammingBot.FrontLeft,
                        TunerConstantsProgrammingBot.FrontRight,
                        TunerConstantsProgrammingBot.BackLeft,
                        TunerConstantsProgrammingBot.BackRight
                    };
            }
            default -> {
                drivetrainConstants = TunerConstantsCompBot.DrivetrainConstants;
                moduleConstants =
                    new SwerveModuleConstants<?, ?, ?>[] {
                        TunerConstantsCompBot.FrontLeft,
                        TunerConstantsCompBot.FrontRight,
                        TunerConstantsCompBot.BackLeft,
                        TunerConstantsCompBot.BackRight
                    };
            }
        }

        // Calculate drivebase dimensions
        trackWidthX = Math.abs(moduleConstants[0].LocationX - moduleConstants[2].LocationX); // front left to back left
        trackWidthY = Math.abs(moduleConstants[0].LocationY - moduleConstants[1].LocationY); // front left to front right

        driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);

        // Calculate kinematic limits
        maxLinearSpeed = moduleConstants[0].SpeedAt12Volts;
        maxOmega = maxLinearSpeed / driveBaseRadius;
    }
}