// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterStuff.AutoShooterRoutineCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.wpilibj.Timer;


public class LimelightSpinFlywheelCommand extends CommandBase {

  private final ShooterSubsystem m_ShooterSubsystem;

  private Timer clock;


  // ++ these are the important variables for this method
  double velocityError;
  double flywheelTargetVelocity;
  boolean isSuccessful;

  /** Creates a new SpinFlywheelCommand. */
  public LimelightSpinFlywheelCommand(ShooterSubsystem subsystem) {
    m_ShooterSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);

    clock = new Timer(); 
  }

  public boolean wasSpinSuccessful() {
    return isSuccessful;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    flywheelTargetVelocity = LimelightSubsystem.giveTargetFlywheelVelocity();
    velocityError = m_ShooterSubsystem.getCurrentFlywheelSpeed() - flywheelTargetVelocity;

    m_ShooterSubsystem.setFlyhweelPIDValues();
    
    clock.reset();
    clock.start();

    isSuccessful = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // m_ShooterSubsystem.setFlywheelSpeed( flywheelTargetVelocity );
    m_ShooterSubsystem.setFlywheelSpeed( ShuffleboardSubsystem.readTargetFlywheelRPM() );

    isSuccessful = Math.abs(velocityError) < Constants.Shooter.flywheelSpeedErrorTolerance;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightSubsystem.continueShooterRoutine = isSuccessful;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isSuccessful || (clock.get() >= Constants.Shooter.flywheelTimeoutTime) || !LimelightSubsystem.continueShooterRoutine);
  }
}
