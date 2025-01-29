package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
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


    public SwerveModule(int driveMotorPort, int steerMotorPort, int steerEncoderPort, boolean isDriveMotorInverted, boolean isSteerMotorInverted) {
        driveMotor = new TalonFX(driveMotorPort);
        driveMotorConfiguration = new TalonFXConfiguration();
        driveMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveMotor.getConfigurator().apply(driveMotorConfiguration.Slot0);
        
        this.steerMotor = new TalonFX(steerMotorPort);
        steerMotorConfiguration = new TalonFXConfiguration();
        steerMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steerMotor.getConfigurator().apply(steerMotorConfiguration);
        
        this.steerEncoder = new CANcoder(steerEncoderPort);

        this.drivePID = new PIDController(Constants.SwerveDrive.DriveMotorPID.P, Constants.SwerveDrive.DriveMotorPID.I, Constants.SwerveDrive.DriveMotorPID.D);
        this.steerPID = new PIDController(Constants.SwerveDrive.SteerMotorPID.P, Constants.SwerveDrive.SteerMotorPID.I, Constants.SwerveDrive.SteerMotorPID.D);
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
        drivePID.setP(Constants.SwerveDrive.DriveMotorPID.P);
        drivePID.setI(Constants.SwerveDrive.DriveMotorPID.I);
        drivePID.setD(Constants.SwerveDrive.DriveMotorPID.D);

        steerPID.setP(Constants.SwerveDrive.SteerMotorPID.P);
        steerPID.setI(Constants.SwerveDrive.SteerMotorPID.I);
        steerPID.setD(Constants.SwerveDrive.SteerMotorPID.D);
    }
}
