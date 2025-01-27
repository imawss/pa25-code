package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    private TalonFX driveMotor;  
    private TalonFX steerMotor;  

    private CANcoder steerEncoder; 

    private PIDController steerPID; 
    private PIDController drivePID; 

    private final double wheelDiameter; 

    public SwerveModule(int driveMotorPort, int steerMotorPort, int steerEncoderPort) {
        this.driveMotor = new TalonFX(driveMotorPort);
        this.steerMotor = new TalonFX(steerMotorPort);
        this.steerEncoder = new CANcoder(steerEncoderPort);

        this.wheelDiameter = Constants.SwerveDrive.WHEEL_DIAMETER;
    }

}
