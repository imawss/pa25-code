// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CustomSwerveModule;

@SuppressWarnings("unused")
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private CANcoder FL;
  private CANcoder FR;
  private CANcoder RL;
  private CANcoder RR;
  private TalonFX steer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    FL = new CANcoder(Constants.CANIds.CANCODER_FL);
    FR = new CANcoder(Constants.CANIds.CANCODER_FR);
    RL = new CANcoder(Constants.CANIds.CANCODER_RL);
    RR = new CANcoder(Constants.CANIds.CANCODER_RR);
    steer = new TalonFX(5);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    System.out.println("Teleop Started");
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    steer.set(0.2);
  }

  @Override    
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    System.out.println("Test başlatıldı!");
  }

  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("FL", FL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("FR", FR.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("RL", RL.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("RR", RR.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void testExit() {
  }
}
