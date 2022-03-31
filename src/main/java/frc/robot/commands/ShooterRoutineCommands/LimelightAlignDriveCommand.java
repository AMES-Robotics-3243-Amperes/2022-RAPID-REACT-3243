// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterRoutineCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.LimelightSubsystem;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;


// ++ ==========================================

/** ++ this class will try to minimize the x-position error of the limelight by rotating the robot.
 * It will read the error from the LimelightSubsystem, and it will RUN THE PID HERE
 */
public class LimelightAlignDriveCommand extends CommandBase {

  private PIDController drivePIDController;
  private Timer clock;

  private final DriveSubsystem m_DriveSubsystem;
  private final LimelightSubsystem m_LimelightSubsystem;

  // ++ this is the important variable here! the entire goal of this class is to minimize this value
  double rotationalOffset;
  // ++ turnVelocity is the speed that's fed into the rotation for the drivetrain
  double turnVelocity; 
  // ++ 
  boolean isSuccessful;

  /** ++ the LimelightSubsystem isn't actually the subsystem for this command,
   * it's just here so we can read the limelight values from it
   */



  /** Creates a new LimelightDriveCommand. */
  public LimelightAlignDriveCommand(DriveSubsystem subsystem, LimelightSubsystem limelightSubsystem) {
    m_DriveSubsystem = subsystem;
    m_LimelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
    addRequirements(m_LimelightSubsystem);

    
    drivePIDController = new PIDController (
      Constants.Shooter.limelightDrivePGain,
      Constants.Shooter.limelightDriveIGain,
      Constants.Shooter.limelightDriveDGain
      );

    
    clock = new Timer();

  }

  /** ++ this method returns true if the robot is successfully aligned */
  public boolean wasAlignSuccessful() {
    return isSuccessful;
  }

  // Called when the command is initially scheduled. 
  @Override 
  public void initialize() {
    isSuccessful = false;

    clock.reset();
    clock.start();

    drivePIDController.reset();

    rotationalOffset = 0.0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotationalOffset = m_LimelightSubsystem.getTargetX();
    turnVelocity = drivePIDController.calculate(rotationalOffset, 0.0);

    m_DriveSubsystem.setReferencesFromWheelSpeeds(0.0, 0.0, turnVelocity);

    // ++ this might be a stupid way of deciding if it's successfully aligned; if it overshoots by a lot when turning,
    // ++ then it'll return true but go past the target. It shouldn't do that if the PID is properly tuned
    isSuccessful = Math.abs(rotationalOffset) < Constants.Shooter.rotationErrorTolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.setReferencesFromWheelSpeeds(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ++ conditions to finish: x error is within acceptable bounds 
    // ++ (( AND the rest of the sub-routines are finished ?? (incase robot got bumped before it was ready to shoot) ))
    // ++ FAILS IF: command times out (can't get within bounds in a certain amount of time)
    return (isSuccessful || (clock.get() >= Constants.Shooter.turnTimeoutTime) );
  }
}
