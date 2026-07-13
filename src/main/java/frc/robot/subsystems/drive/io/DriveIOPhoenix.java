package frc.robot.subsystems.drive.io;

import frc.lib.swerve.SwerveDriveCoast;
import frc.robot.subsystems.drive.DriveConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class DriveIOPhoenix extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
    // Signals
    private final BaseStatusSignal[] drivePositionSignals = new BaseStatusSignal[4];
    private final BaseStatusSignal[] driveVelocitySignals = new BaseStatusSignal[4];
    private final StatusSignal<Angle> gyroYaw;

    // Requests
    private final ApplyRobotSpeeds APPLY_ROBOT_SPEEDS =
        new ApplyRobotSpeeds()
            .withCenterOfRotation(DriveConstants.centerOfRotation)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withDesaturateWheelSpeeds(true);

    private final SysIdSwerveTranslation RUN_CHARACTERIZATION = new SysIdSwerveTranslation();

    public DriveIOPhoenix(SwerveModuleConstants<?, ?, ?>... modules) {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            DriveConstants.drivetrainConstants,
            modules);

        gyroYaw = super.getPigeon2().getYaw();

        for (int i = 0; i < 4; i++) {
            TalonFX driveMotor = super.getModule(i).getDriveMotor();

            drivePositionSignals[i] = driveMotor.getPosition();
            driveVelocitySignals[i] = driveMotor.getVelocity();
        }
    }

    public DriveIOPhoenix() {
        this(DriveConstants.moduleConstants);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        // Refresh all signals
        BaseStatusSignal.refreshAll(drivePositionSignals);
        BaseStatusSignal.refreshAll(driveVelocitySignals);
        BaseStatusSignal.refreshAll(gyroYaw);

        // Update drive inputs
        inputs.fromSwerveDriveState(super.getStateCopy());
        
        for (int i = 0; i < 4; i++) {
            inputs.drivePositionsRad[i] =
                Units.rotationsToRadians(drivePositionSignals[i].getValueAsDouble());

            inputs.driveVelocitiesRadPerSec[i] =
                Units.rotationsToRadians(driveVelocitySignals[i].getValueAsDouble());
        }
        inputs.gyroAngle = Rotation2d.fromDegrees(gyroYaw.getValueAsDouble());
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void resetAngle(Rotation2d angle) {
        super.resetRotation(angle);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        double newTimestamp = Utils.fpgaToCurrentTime(timestamp);

        if (stdDevs != null) {
            super.addVisionMeasurement(pose, newTimestamp, stdDevs);
        } else {
            super.addVisionMeasurement(pose, newTimestamp);
        }
    }

    @Override
    public void stopWithX() {
        super.setControl(new SwerveDriveBrake());
    }

    @Override
    public void coast() {
        super.setControl(new SwerveDriveCoast());
    }

    @Override
    public void runVelocity(ChassisSpeeds speeds) {
        super.setControl(APPLY_ROBOT_SPEEDS.withSpeeds(speeds));
    }

    @Override
    public void runCharacterization(double output) {
        super.setControl(RUN_CHARACTERIZATION.withVolts(output));
    }
}
