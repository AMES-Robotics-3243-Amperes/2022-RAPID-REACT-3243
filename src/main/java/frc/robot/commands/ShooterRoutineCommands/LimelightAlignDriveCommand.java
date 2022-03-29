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

  // ++ this is the important variable here! the entire goal of this class is to minimize this value
  double rotationalError;
  // ++ turnVelocity is the speed that's fed into the rotation for the drivetrain
  double turnVelocity; 
  // ++ 
  boolean isSuccessful;

  /** ++ the LimelightSubsystem isn't actually the subsystem for this command,
   * it's just here so we can read the limelight values from it
   */
  LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();



  /** Creates a new LimelightDriveCommand. */
  public LimelightAlignDriveCommand(DriveSubsystem subsystem) {
    m_DriveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);

    
    drivePIDController = new PIDController (
      Constants.Shooter.limelightDrivePGain,
      Constants.Shooter.limelightDriveIGain,
      Constants.Shooter.limelightDriveDGain
      );

    
    clock = new Timer();

  }

  public boolean wasAlignSuccessful() {
    if (isSuccessful) {
      return true;
    }
    else {
      return false;
    }
  }

  // Called when the command is initially scheduled. 
  @Override 
  public void initialize() {
    // ++ ASK DANIAL-- should this be in initialize() or end()? For a command group, does initialize() get scheduled(therefore run)
    // ++ every time the command group is scheduled?
    isSuccessful = false;

    clock.reset();
    clock.start();


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rotationalError = m_LimelightSubsystem.getTargetX();
    turnVelocity = drivePIDController.calculate(rotationalError);

    m_DriveSubsystem.setReferencesFromWheelSpeeds(0.0, 0.0, turnVelocity);

    if ( Math.abs(rotationalError) < Constants.Shooter.rotationErrorTolerance) {
      isSuccessful = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivePIDController.reset();
    rotationalError = 0.0;
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
