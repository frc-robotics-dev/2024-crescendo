package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class DriveViz {
    private final boolean logModules = false;

    private final LoggedMechanism2d[] moduleMechanisms = new LoggedMechanism2d[4];
    private final LoggedMechanismLigament2d[] moduleSpeeds = new LoggedMechanismLigament2d[4];
    private final LoggedMechanismLigament2d[] moduleDirections = new LoggedMechanismLigament2d[4];

    public DriveViz() {
        for (int i = 0; i < 4; i++) {
            moduleMechanisms[i] = new LoggedMechanism2d(1, 1);

            moduleSpeeds[i] =
                moduleMechanisms[i]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new LoggedMechanismLigament2d("Speed", 0.5, 0));

            moduleDirections[i] =
                moduleMechanisms[i]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(new LoggedMechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite)));
        }
    }

    public void update(SwerveDriveState state) {
        // Get inputs
        Pose2d pose = state.Pose;
        ChassisSpeeds robotRelativeSpeeds = state.Speeds;
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());

        // Log robot velocities
        Logger.recordOutput("Drive/Speed", Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond));
        Logger.recordOutput("Drive/Field Relative Speeds", fieldRelativeSpeeds);

        // Log modules
        if (!logModules) {
            return;
        }

        for (int i = 0; i < 4; i++) {
            SwerveModuleState moduleState = state.ModuleStates[i];

            moduleSpeeds[i].setAngle(moduleState.angle);
            moduleDirections[i].setAngle(moduleState.angle);
            moduleSpeeds[i].setLength(moduleState.speedMetersPerSecond / DriveConstants.maxLinearSpeed);
        }

        Logger.recordOutput("Drive/Modules/Viz/FrontLeft", moduleMechanisms[0]);
        Logger.recordOutput("Drive/Modules/Viz/FrontRight", moduleMechanisms[1]);
        Logger.recordOutput("Drive/Modules/Viz/BackLeft", moduleMechanisms[2]);
        Logger.recordOutput("Drive/Modules/Viz/BackRight", moduleMechanisms[3]);
    }
}