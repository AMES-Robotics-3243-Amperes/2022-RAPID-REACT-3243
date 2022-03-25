// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.JoyUtil;

public class TeleopPIDDriveCommand extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private final JoyUtil controller;

  /** Creates a new TeleopPIDCommand. */
  public TeleopPIDDriveCommand(DriveSubsystem subsystem, JoyUtil controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = subsystem;
    addRequirements(m_DriveSubsystem);
    this.controller = controller;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.zeroPreviousFiltered();
    m_DriveSubsystem.getShuffleboardPID();
    m_DriveSubsystem.setPIDValues(m_DriveSubsystem.pGain.getDouble(1.0),m_DriveSubsystem.iGain.getDouble(0.0),m_DriveSubsystem.dGain.getDouble(0.0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    MecanumDriveWheelSpeeds wheelSpeeds = m_DriveSubsystem.getWheelSpeeds(
      controller.getDriveXWithAdjustments(),
      controller.getDriveYWithAdjustments(),
      controller.getRotationWithAdjustments()
      );
    
    m_DriveSubsystem.setVelocityReference(wheelSpeeds);
    
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
