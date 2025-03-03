package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CustomSwerveSubsystem extends SubsystemBase {
    public final CustomSwerveModule FLModule = new CustomSwerveModule(
            Constants.CANIds.DRIVE_FL,
            Constants.CANIds.STEER_FL,
            Constants.CANIds.CANCODER_FL,
            Constants.SwerveDrive.kDriveFLInverted,
            Constants.SwerveDrive.kSteerFLInverted,
            new Rotation2d(Constants.SwerveDrive.kFLOffset));

    public final CustomSwerveModule FRModule = new CustomSwerveModule(
            Constants.CANIds.DRIVE_FR,
            Constants.CANIds.STEER_FR,
            Constants.CANIds.CANCODER_FR,
            Constants.SwerveDrive.kDriveFRInverted,
            Constants.SwerveDrive.kSteerFRInverted,
            new Rotation2d(Constants.SwerveDrive.kFROffset));

    public final CustomSwerveModule RLModule = new CustomSwerveModule(
            Constants.CANIds.DRIVE_RL,
            Constants.CANIds.STEER_RL,
            Constants.CANIds.CANCODER_RL,
            Constants.SwerveDrive.kDriveRLInverted,
            Constants.SwerveDrive.kSteerRLInverted,
            new Rotation2d(Constants.SwerveDrive.kRLOffset));

    public final CustomSwerveModule RRModule = new CustomSwerveModule(
            Constants.CANIds.DRIVE_RR,
            Constants.CANIds.STEER_RR,
            Constants.CANIds.CANCODER_RR,
            Constants.SwerveDrive.kDriveRRInverted,
            Constants.SwerveDrive.kSteerRRInverted,
            new Rotation2d(Constants.SwerveDrive.kRROffset));

    private Pigeon2 gyro;
    public final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometer;

    public CustomSwerveSubsystem() {
        kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.SwerveDrive.kWheelBase / 2, -Constants.SwerveDrive.kTrackWidth / 2),
                new Translation2d(Constants.SwerveDrive.kWheelBase / 2, Constants.SwerveDrive.kTrackWidth / 2),
                new Translation2d(-Constants.SwerveDrive.kWheelBase / 2, -Constants.SwerveDrive.kTrackWidth / 2),
                new Translation2d(Constants.SwerveDrive.kWheelBase / 2, Constants.SwerveDrive.kTrackWidth / 2));

        gyro = new Pigeon2(Constants.CANIds.PIGEON);

        SwerveModulePosition[] modulePositions = {
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d())
        };

        odometer = new SwerveDriveOdometry(kinematics, getGyroAngle(), modulePositions);
        FLModule.resetEncoders();
        FRModule.resetEncoders();
        RLModule.resetEncoders();
        RRModule.resetEncoders();

        if (gyro.isConnected()) {
            resetGyro();
        }
    }

    @Override
    public void periodic() {
        odometer.update(getGyroAngle(),
                getModulePositions(new CustomSwerveModule[] { FLModule, FRModule, RLModule, RRModule }));
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    private Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue().in(Rotation));
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getYaw().getValue().in(Rotation), 360);
    }

    private void resetGyro() {
        gyro.reset();
        gyro.setYaw(0);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
        // TODO: Dashboard
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
        // TODO: Dashboard
    }

    public SwerveModulePosition[] getModulePositions(CustomSwerveModule[] modules) {
        SwerveModulePosition[] tModulePositions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            tModulePositions[i] = modules[i].getOdometerPosition();
        }

        return tModulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                Constants.SwerveDrive.kPhysicalMaxSpeedMetersPerSecond);
        FLModule.setDesiredState(desiredStates[0]);
        FRModule.setDesiredState(desiredStates[1]);
        RLModule.setDesiredState(desiredStates[2]);
        RRModule.setDesiredState(desiredStates[3]);
    }

    public void stopModules() {
        FLModule.stopModule();
        FRModule.stopModule();
        RLModule.stopModule();
        RRModule.stopModule();
    }

}
