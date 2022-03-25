// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeIndexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexerSubsystem;

public class AcceptCommand extends CommandBase {

  private final IntakeIndexerSubsystem m_subsystem;
  private final double m_rotations;

  /** Creates a new AcceptCommand. */
  public AcceptCommand(IntakeIndexerSubsystem subsystem, double rotations) {
    m_subsystem = subsystem;
    m_rotations = rotations;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.stepIndexer(m_rotations);
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
    return true;
  }
}
