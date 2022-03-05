// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexerSubsystem;

public class RebuffCommand extends CommandBase {

  private final IntakeIndexerSubsystem m_subsystem;
  private final double m_rotations;
  private final double m_intakeSpeed;
  private final double m_duration;
  private final Timer timer;

  /** Creates a new RebuffCommand. */
  public RebuffCommand(IntakeIndexerSubsystem subsystem, double rotations, double intakeSpeed, double duration) {
    m_subsystem = subsystem;
    m_rotations = rotations;
    m_intakeSpeed = intakeSpeed;
    m_duration = duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    m_subsystem.stepIndexer(m_rotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setIntakeSpeed(m_intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > m_duration);
  }
}
