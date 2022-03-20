// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.JoyUtil;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem m_ShooterSubsystem;
    private final JoyUtil joystick;

  public ShooterCommand(ShooterSubsystem subsystem, JoyUtil secondaryController) {
    m_ShooterSubsystem = subsystem;
    joystick = secondaryController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  double controllerInput = 0;
  @Override
  public void execute() {
    if (joystick.getAButton()) {
      controllerInput += 0.1;
      m_ShooterSubsystem.setHoodAngle(controllerInput);
    }
    else if (joystick.getBButton()) {
      controllerInput -= 0.1;
      m_ShooterSubsystem.setHoodAngle(controllerInput);
    }
    else if (joystick.getRightStickButtonPressed())  {
      m_ShooterSubsystem.setHoodAngle(2);
      controllerInput = 0;
    }
    else if (joystick.getLeftStickButtonPressed()) {
      m_ShooterSubsystem.setHoodAngle(4);
      controllerInput = 0;
    }
    else if (joystick.getLeftBumperPressed()) {
      m_ShooterSubsystem.setHoodAngle(6);
      controllerInput = 0;
    }
    else {
      m_ShooterSubsystem.setHoodAngle(controllerInput);
    }
    

     
    if (joystick.getYButton()) { 
      m_ShooterSubsystem.setFlywheelSpeed(3000);
      SmartDashboard.putNumber("flywheel speed", m_ShooterSubsystem.getFlywheelSpeed());
    }
    else {
      m_ShooterSubsystem.stopFlywheel();
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
