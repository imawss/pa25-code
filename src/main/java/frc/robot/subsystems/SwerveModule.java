package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private TalonFX driveMotor;
    private TalonFXConfiguration driveMotorConfiguration;

    private TalonFX steerMotor;
    private TalonFXConfiguration steerMotorConfiguration;

    private CANcoder steerEncoder;

    private PIDController steerPID;
    private PIDController drivePID;

    public SwerveModule(int driveMotorPort, int steerMotorPort, int steerEncoderPort, boolean isDriveMotorInverted,
            boolean isSteerMotorInverted) {
        driveMotor = new TalonFX(driveMotorPort);
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.MotorOutput.Inverted = isDriveMotorInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        driveMotorConfiguration.Feedback.SensorToMechanismRatio = Constants.SwerveDrive.kGearRatio;
        driveMotor.getConfigurator().apply(driveMotorConfiguration);

        this.steerMotor = new TalonFX(steerMotorPort);
        steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.MotorOutput.Inverted = isSteerMotorInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        steerMotor.getConfigurator().apply(steerMotorConfiguration);

        this.steerEncoder = new CANcoder(steerEncoderPort);

        this.drivePID = new PIDController(Constants.SwerveDrive.DriveMotorPID.kP,
                Constants.SwerveDrive.DriveMotorPID.kI, Constants.SwerveDrive.DriveMotorPID.kD);
        this.steerPID = new PIDController(Constants.SwerveDrive.SteerMotorPID.kP,
                Constants.SwerveDrive.SteerMotorPID.kI, Constants.SwerveDrive.SteerMotorPID.kD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getDrivePosition() {
        return Conversions.rotationsToMeters(driveMotor.getPosition().getValue().in(Rotations),
                Constants.SwerveDrive.kWheelCircumference);
    }

    public double getDriveVelocity() {
        return Conversions.RPSToMPS(driveMotor.getVelocity().getValue().in(RotationsPerSecond),
                Constants.SwerveDrive.kWheelCircumference);
    }

    public double getSteerAngle() {
        return steerEncoder.getAbsolutePosition().getValue().in(Rotations)
                * Constants.SwerveDrive.kTurningEncoderRot2Rad;
    }

    public double getSteerVelocity() {
        return Conversions.RPSToMPS(steerEncoder.getVelocity().getValue().in(RotationsPerSecond),
                Constants.SwerveDrive.kWheelCircumference);
    }

    public void stopModule() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        steerEncoder.setPosition(steerEncoder.getAbsolutePosition().getValue());
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stopModule();
            return;
        }
        state.optimize(new Rotation2d(getSteerAngle()));
        double driveOutput = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        driveMotor.set(driveOutput);
        double steerOutput = steerPID.calculate(getSteerAngle(), state.angle.getRadians());
        steerMotor.set(steerOutput);
        SmartDashboard.putString("Swerve[" + steerEncoder.getDeviceID() + "] state", state.toString());
    }

    public void configurePIDForAutonomous(double kPD, double kID, double kDD, double kPS, double kIS, double kDS) {
        drivePID.setP(kPD);
        drivePID.setI(kID);
        drivePID.setD(kDD);

        steerPID.setP(kPS);
        steerPID.setI(kIS);
        steerPID.setD(kDS);
    }

    public void resetPIDToDefault() {
        drivePID.setP(Constants.SwerveDrive.DriveMotorPID.kP);
        drivePID.setI(Constants.SwerveDrive.DriveMotorPID.kI);
        drivePID.setD(Constants.SwerveDrive.DriveMotorPID.kD);

        steerPID.setP(Constants.SwerveDrive.SteerMotorPID.kP);
        steerPID.setI(Constants.SwerveDrive.SteerMotorPID.kI);
        steerPID.setD(Constants.SwerveDrive.SteerMotorPID.kD);
    }
}
