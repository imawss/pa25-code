package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.DeviceIdentifier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private TalonFX driveMotor;  
    private TalonFXConfiguration driveMotorConfiguration;

    private TalonFX steerMotor;  
    private TalonFXConfiguration steerMotorConfiguration;

    private CANcoder steerEncoder; 

    private PIDController steerPID; 
    private PIDController drivePID; 

    private final double wheelDiameter; 

    public SwerveModule(int driveMotorPort, int steerMotorPort, int steerEncoderPort, boolean isDriveMotorInverted, boolean isSteerMotorInverted) {
        driveMotor = new TalonFX(driveMotorPort);
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveMotor.getConfigurator().apply(driveMotorConfiguration);
        
        this.steerMotor = new TalonFX(steerMotorPort);
        steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerMotor.getConfigurator().apply(steerMotorConfiguration);
        
        this.steerEncoder = new CANcoder(steerEncoderPort);

        this.wheelDiameter = Constants.SwerveDrive.WHEEL_DIAMETER;
        this.drivePID = new PIDController(Constants.SwerveDrive.DriveMotorPID.P, Constants.SwerveDrive.DriveMotorPID.I, Constants.SwerveDrive.DriveMotorPID.D);
        this.steerPID = new PIDController(Constants.SwerveDrive.SteerMotorPID.P, Constants.SwerveDrive.SteerMotorPID.I, Constants.SwerveDrive.SteerMotorPID.D);
    }

}
