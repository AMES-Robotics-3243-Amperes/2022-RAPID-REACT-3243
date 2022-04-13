// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeIndexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousSpintakeCommand extends CommandBase {
  private final IntakeIndexerSubsystem m_subsystem;
  private final double m_duration;
  private final Timer timer;
  /** Creates a new AutonomousSpintakeCommand. */
  public AutonomousSpintakeCommand(IntakeIndexerSubsystem subsystem, double duration) {
    m_subsystem = subsystem;
    m_duration = duration;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setIntakeSpeed(Constants.IntakeIndexer.intakeSpeed);
    m_subsystem.setIndexerSpeed(Constants.IntakeIndexer.indexSpeed);
    //m_subsystem2.setVelocityReference(Constants.DriveTrain.maxWheelSpeed/-5, Constants.DriveTrain.maxWheelSpeed/-5, Constants.DriveTrain.maxWheelSpeed/-5, Constants.DriveTrain.maxWheelSpeed/-5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntakeSpeed(0.0);
    m_subsystem.setIndexerSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= m_duration);
  }
}
