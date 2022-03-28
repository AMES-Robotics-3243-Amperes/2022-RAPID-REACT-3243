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

    double hoodAngle;

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

    // m_ShooterSubsystem.setHoodPIDValues();
    hoodAngle = 0.0;
    m_ShooterSubsystem.zeroHoodEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.


  @Override
  public void execute() {
    if (joystick.getStartButton()) {
      hoodAngle += 0.5;
    }
    if (joystick.getBackButton()) {
      hoodAngle -= 0.5;
    }
    
    m_ShooterSubsystem.setHoodAngle(hoodAngle);
    SmartDashboard.putNumber("hood target", hoodAngle);
    SmartDashboard.putNumber("hood actual", m_ShooterSubsystem.getHoodAngle());


    

    // ++ this sets the speed of the flywheel
    if (joystick.getYButton()) { 
      double speed = 1.0;
      m_ShooterSubsystem.setFlywheelSpeed(speed);
    } else {
      m_ShooterSubsystem.stopFlywheel();
    }

    m_ShooterSubsystem.setFlywheelSpeed(2.0 * joystick.getRightTriggerAxis());
    
    
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
