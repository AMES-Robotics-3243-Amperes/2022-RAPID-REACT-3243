// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterRoutineCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;




public class MoveHoodCommand extends CommandBase {


  /**++ this is the subsystem this command controls */
  private final ShooterSubsystem m_ShooterSubsystem;
  /** ++ the LimelightSubsystem isn't actually the subsystem for this command,
  * it's just here so we can read the limelight values from it
  */
  private LimelightSubsystem m_LimelightSubsystem;

  // ++ these are super important variables for this class! the goal is to minimize the error between the current and the target
  double currentHoodAngle;
  double targetHoodAngle;
  boolean isSuccessful;

  private Timer clock;
  private PIDController hoodPIDController;




  /** Creates a new MoveHoodCommand. */
  public MoveHoodCommand(ShooterSubsystem subsystem, LimelightSubsystem limelightSubsystem) {
    m_ShooterSubsystem = subsystem;
    m_LimelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ShooterSubsystem);
    addRequirements(m_LimelightSubsystem);
    clock = new Timer();

    hoodPIDController = new PIDController(
      Constants.Shooter.hoodPGain,
      Constants.Shooter.hoodIGain,
      Constants.Shooter.hoodDGain
    );
  }

  /** ++ this method returns true if the hood is successfully in place */
  public boolean wasHoodAdjustSuccessful() {
    return isSuccessful;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.setHoodPIDValues();
    clock.reset();
    clock.start();

    isSuccessful = false;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentHoodAngle = m_ShooterSubsystem.getHoodAngle();
    targetHoodAngle = m_LimelightSubsystem.findTargetHoodAngle();

    double nextHoodOutput = hoodPIDController.calculate(currentHoodAngle, targetHoodAngle);

    m_ShooterSubsystem.setHoodAngle(nextHoodOutput);

    isSuccessful = Math.abs(currentHoodAngle - targetHoodAngle) < Constants.Shooter.hoodErrorTolerance;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isSuccessful || (clock.get() >= Constants.Shooter.hoodTimeoutTime) );
  }
}
