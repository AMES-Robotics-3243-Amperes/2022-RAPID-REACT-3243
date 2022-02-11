// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndexerSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetIntakeCommand extends CommandBase {
  private final IntakeIndexerSubsystem m_intake;
  private final double targetPos;

  /** Creates a new DropIntakeCommand. */
  public SetIntakeCommand(IntakeIndexerSubsystem intake, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    targetPos = position;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
    return false;
  }
}
