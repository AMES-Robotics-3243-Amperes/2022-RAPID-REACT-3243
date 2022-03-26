// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeIndexer;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndexerSubsystem;

public class AcceptCommand extends CommandBase {

  private final IntakeIndexerSubsystem m_subsystem;
  private Timer timer;

  /** Creates a new AcceptCommand. */
  public AcceptCommand(IntakeIndexerSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    m_subsystem.setIntakeSpeed(Constants.IntakeIndexer.acceptSpeed);
    m_subsystem.stepIndexer(Constants.IntakeIndexer.acceptRotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ++ in theis method, we want to stop both the intake and indexer
    m_subsystem.setIntakeSpeed(0.0);
    m_subsystem.stepIndexer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > Constants.IntakeIndexer.acceptDuration);
  }
}