package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.subsystems.CustomSwerveSubsystem;

public class RobotContainer {

  private final CustomSwerveSubsystem swerveSubsystem = new CustomSwerveSubsystem();
  private final Joystick joystick = new Joystick(0);
  Supplier<Double> xSpdSupplier;
  Supplier<Double> ySpdSupplier;
  Supplier<Double> turningSpdSupplier;
  Supplier<Boolean> fieldOrientedSupplier;

  private final SwerveJoystickCommand swerveJoystickCommand;

  public RobotContainer() {
    swerveJoystickCommand = new SwerveJoystickCommand(swerveSubsystem, xSpdSupplier, ySpdSupplier, turningSpdSupplier,
        fieldOrientedSupplier);

    configureBindings();
  }

  private void configureBindings() {
    xSpdSupplier = () -> joystick.getX();
    ySpdSupplier = () -> joystick.getY();
    turningSpdSupplier = () -> joystick.getZ();
    fieldOrientedSupplier = () -> true;
  }

  public void teleopInit() {
    swerveJoystickCommand.initialize();
  }

  public void teleopPeriodic() {
    swerveJoystickCommand.execute();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
