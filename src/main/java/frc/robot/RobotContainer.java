package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.CustomSwerveSubsystem;

public class RobotContainer {

  private final CustomSwerveSubsystem swerveSubsystem = new CustomSwerveSubsystem();
  private final PS4Controller driverJoystick = new PS4Controller(0);

  public RobotContainer() {
    configureBindings();
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
        swerveSubsystem,
        () -> driverJoystick.getLeftX(),
        () -> driverJoystick.getLeftY(),
        () -> driverJoystick.getRightX(),
        true));
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
