package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class SwerveSubsystem {
    public final SwerveModule FLModule = new SwerveModule(
            Constants.CANIds.DRIVE_FL,
            Constants.CANIds.STEER_FL,
            Constants.CANIds.CANCODER_FL,
            Constants.SwerveDrive.kDriveFLInverted,
            Constants.SwerveDrive.kSteerFLInverted);

    public final SwerveModule FRModule = new SwerveModule(
            Constants.CANIds.DRIVE_FR,
            Constants.CANIds.STEER_FR,
            Constants.CANIds.CANCODER_FR,
            Constants.SwerveDrive.kDriveFRInverted,
            Constants.SwerveDrive.kSteerFRInverted);

    public final SwerveModule RRModule = new SwerveModule(
            Constants.CANIds.DRIVE_RL,
            Constants.CANIds.STEER_RL,
            Constants.CANIds.CANCODER_RL,
            Constants.SwerveDrive.kDriveRLInverted,
            Constants.SwerveDrive.kSteerRLInverted);

    public final SwerveModule RLModule = new SwerveModule(
            Constants.CANIds.DRIVE_RL,
            Constants.CANIds.STEER_RL,
            Constants.CANIds.CANCODER_RL,
            Constants.SwerveDrive.kDriveRLInverted,
            Constants.SwerveDrive.kSteerRLInverted);

    private Pigeon2 gyro;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public SwerveSubsystem() {
        kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.SwerveDrive.kWheelBase / 2, -Constants.SwerveDrive.kTrackWidth / 2),
                new Translation2d(Constants.SwerveDrive.kWheelBase / 2, Constants.SwerveDrive.kTrackWidth / 2),
                new Translation2d(-Constants.SwerveDrive.kWheelBase / 2, -Constants.SwerveDrive.kTrackWidth / 2),
                new Translation2d(-Constants.SwerveDrive.kWheelBase / 2, Constants.SwerveDrive.kTrackWidth / 2));

        gyro = new Pigeon2(Constants.CANIds.PIGEON);

        SwerveModulePosition[] modulePositions = {
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d()),
                new SwerveModulePosition(0.0, new Rotation2d())
        };

        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), modulePositions);
    }

    private Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue().in(Rotation));
    }
}
