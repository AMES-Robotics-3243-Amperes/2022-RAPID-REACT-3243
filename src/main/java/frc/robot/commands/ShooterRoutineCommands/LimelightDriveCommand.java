// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterRoutineCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;


// ++ ==========================================

/** ++ this class will try to minimize the x-position error of the limelight by rotating the robot.
 * It will read the error from the LimelightSubsystem, and it will RUN THE PID HERE
 */
public class LimelightDriveCommand extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;

  /** Creates a new LimelightDriveCommand. */
  public LimelightDriveCommand(DriveSubsystem subsystem) {
    m_DriveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    m_DriveSubsystem.getShuffleboardPID();

    m_DriveSubsystem.setPIDValues(
      m_DriveSubsystem.pGain.getDouble( Constants.DriveTrain.teleopPGain ),
      m_DriveSubsystem.iGain.getDouble( Constants.DriveTrain.teleopIGain ),
      m_DriveSubsystem.dGain.getDouble( Constants.DriveTrain.teleopDGain )
      );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ++ conditions to finish: x error is within acceptable bounds (( AND the rest of the sub-routines are finished ? ))
    return false;
  }
}
