package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class CustomSwerveModule extends SubsystemBase {
    private TalonFX driveMotor;
    private TalonFXConfiguration driveMotorConfiguration;

    private TalonFX steerMotor;
    private TalonFXConfiguration steerMotorConfiguration;

    private CANcoder steerEncoder;
    private final Rotation2d CANCoderOffset;

    public CustomSwerveModule(int driveMotorPort, int steerMotorPort, int steerEncoderPort,
            boolean isDriveMotorInverted,
            boolean isSteerMotorInverted, Rotation2d CANCoderOffset) {

        this.steerEncoder = new CANcoder(steerEncoderPort);
        this.CANCoderOffset = CANCoderOffset;
        
        driveMotor = new TalonFX(driveMotorPort);
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.MotorOutput.Inverted = isDriveMotorInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        driveMotorConfiguration.Feedback.SensorToMechanismRatio = Constants.SwerveDrive.kGearRatio;
        driveMotorConfiguration.Slot0.kP = Constants.SwerveDrive.DriveMotorPID.kP;
        driveMotorConfiguration.Slot0.kI = Constants.SwerveDrive.DriveMotorPID.kI;
        driveMotorConfiguration.Slot0.kD = Constants.SwerveDrive.DriveMotorPID.kD;
        driveMotor.getConfigurator().apply(driveMotorConfiguration);

        this.steerMotor = new TalonFX(steerMotorPort);
        steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.MotorOutput.Inverted = isSteerMotorInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        steerMotorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        steerMotorConfiguration.Slot0.kP = Constants.SwerveDrive.SteerMotorPID.kP;
        steerMotorConfiguration.Slot0.kI = Constants.SwerveDrive.SteerMotorPID.kI;
        steerMotorConfiguration.Slot0.kD = Constants.SwerveDrive.SteerMotorPID.kD;
        steerMotorConfiguration.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();
        steerMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerMotor.getConfigurator().apply(steerMotorConfiguration);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getSteerAngle());
    }

    public SwerveModulePosition getOdometerPosition() {
        return new SwerveModulePosition(getDrivePosition(), getSteerAngle());
    }

    public double getDrivePosition() {
        double wheelRotations = driveMotor.getPosition().getValue().in(Rotations);
        return wheelRotations * Constants.SwerveDrive.kDriveEncoderRot2Meter;
    }

    public double getDriveVelocity() {
        double wheelRPS = driveMotor.getVelocity().getValue().in(RotationsPerSecond);
        return wheelRPS * Constants.SwerveDrive.kDriveEncoderRot2Meter;
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValue().in(Rotations));
    }

    public double getSteerVelocity() {
        return Conversions.RPSToMPS(steerEncoder.getVelocity().getValue().in(RotationsPerSecond),
                Constants.SwerveDrive.kWheelCircumference);
    }

    public void stopModule() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    private void configureSteerEncoder() {
        Rotation2d absolutePosition = Rotation2d.fromRotations(steerEncoder.getAbsolutePosition().getValue().in(Degree))
                .minus(CANCoderOffset);
        steerEncoder.setPosition(absolutePosition.getRotations());
        steerMotor.setPosition(absolutePosition.getRotations());
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        configureSteerEncoder();
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stopModule();
            return;
        }

        state.optimize(getSteerAngle());

        double targetVelocityRPS = state.speedMetersPerSecond / Constants.SwerveDrive.kDriveEncoderRot2Meter;
        driveMotor.setControl(new VelocityVoltage(targetVelocityRPS));
        steerMotor.setControl(new PositionVoltage(state.angle.getRotations()));
    }

    public void testSpeed(double driveSpeed, double steerSpeed) {
        driveMotor.set(driveSpeed);
        steerMotor.set(steerSpeed);
    }
}
