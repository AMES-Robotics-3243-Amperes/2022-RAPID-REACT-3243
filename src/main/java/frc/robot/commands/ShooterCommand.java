// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.JoyUtil;
import frc.robot.subsystems.ShooterSubsystem;
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
  public void initialize() {
    m_ShooterSubsystem.setFlyhweelPIDValues();
  }

  // Called every time the scheduler runs while the command is scheduled.


  @Override
  public void execute() {
    double hoodAngle = 0;
    if (joystick.getAButton()) {
      hoodAngle += 0.1;
    }
    else if (joystick.getBButton()) {
      hoodAngle -= 0.1;
    }
    else if (joystick.getRightStickButtonPressed())  {
      hoodAngle = 0;
    }
    else {
      m_ShooterSubsystem.setHoodAngle(hoodAngle);
    }
    

    // ++ this sets the speed of the flywheel
    if (joystick.getYButton()) { 
      double speed = 1.0;
      m_ShooterSubsystem.setFlywheelSpeed(speed);
    } else {
      m_ShooterSubsystem.stopFlywheel();
    }

    m_ShooterSubsystem.setFlywheelSpeed(2 * joystick.getRightTriggerAxis());
    
    
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
