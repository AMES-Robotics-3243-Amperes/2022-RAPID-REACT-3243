// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterStuff;

import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ShooterCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ShooterSubsystem m_ShooterSubsystem;
    private final HoodSubsystem m_HoodSubsystem;
    private final JoyUtil joystick;


  public ShooterCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, JoyUtil secondaryController) {
    m_ShooterSubsystem = shooterSubsystem;
    m_HoodSubsystem = hoodSubsystem;
    joystick = secondaryController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
    addRequirements(m_HoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setFlyhweelPIDValues();

  }

  // Called every time the scheduler runs while the command is scheduled.
  

  @Override
  // ££ Hood Angle Code
  public void execute() {
  
    // ++ this sets the speed of the flywheel
    if ( joystick.getRightTriggerAxis() > 0.5 ) {
      m_ShooterSubsystem.setFlywheelSpeed( Constants.Shooter.flywheelRPMFromLaunchPad );
      m_HoodSubsystem.setServoPositionFromHoodAngle( 60.0 );
    } else {
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
