package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CustomSwerveSubsystem;

public class SwerveJoystickCommand extends Command {
    private final CustomSwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xInput, yInput, turningInput;
    private final Boolean fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCommand(CustomSwerveSubsystem swerveSubsystem,
            DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier turningInput,
            Boolean fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xInput = xInput;
        this.yInput = yInput;
        this.turningInput = turningInput;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(Constants.SwerveDrive.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(Constants.SwerveDrive.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(Constants.SwerveDrive.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = xInput.getAsDouble();
        double ySpeed = yInput.getAsDouble();
        double turningSpeed = turningInput.getAsDouble();

        xSpeed = Math.abs(xSpeed) > Constants.DriverConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.DriverConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > Constants.DriverConstants.kDeadband ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * Constants.SwerveDrive.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.SwerveDrive.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.SwerveDrive.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        SwerveModuleState[] moduleStates = swerveSubsystem.kinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
