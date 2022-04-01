// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutnomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

public class LookAtCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private Pose2d m_target;
  private boolean shuffleboard;
  /** Creates a new LookAtCommand using the target position from shuffleboard. */
  public LookAtCommand(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    shuffleboard = true;
  }

  /** Creates a new LookAtCommand using the supplied target position */
  public LookAtCommand(DriveSubsystem subsystem, Pose2d target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    m_target = target;
    shuffleboard = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (shuffleboard) {
      m_target = ShuffleboardSubsystem.getTargetPose();
    }
    m_subsystem.lookAt(m_target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atTargetPosition();
  }
}
