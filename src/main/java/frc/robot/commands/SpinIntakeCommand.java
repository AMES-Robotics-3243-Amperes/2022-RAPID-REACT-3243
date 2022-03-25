// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.Constants;

import frc.robot.JoyUtil;

public class SpinIntakeCommand extends CommandBase {

  private final IntakeIndexerSubsystem m_subsystem;
  private final JoyUtil secondaryController;

  /** Creates a new SpinIntakeCommand. */
  public SpinIntakeCommand(IntakeIndexerSubsystem subsystem, JoyUtil controller) {
    m_subsystem = subsystem;
    secondaryController = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isIntakeOn = secondaryController.getLeftBumper();

    if (isIntakeOn == true) {
      m_subsystem.setIntakeSpeed(1);
      m_subsystem.setIndexerSpeed(-1);
    } else {
      m_subsystem.setIntakeSpeed(0);
      m_subsystem.setIndexerSpeed(0);
    }
    // m_subsystem.setIntakeSpeed();
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
