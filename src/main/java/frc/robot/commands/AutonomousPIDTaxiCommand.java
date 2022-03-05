// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
//Taxi means moving out of the tarmac with the autonomous PIDs. Thanks Gabe
public class AutonomousPIDTaxiCommand extends CommandBase {
  private DriveSubsystem m_subsystem;
  /** Creates a new AutonomousPIDTaxi. */
  public AutonomousPIDTaxiCommand(DriveSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.getShuffleboardPID();
    m_subsystem.setPID(m_subsystem.pGain.getDouble(1.0),m_subsystem.iGain.getDouble(0.0),m_subsystem.dGain.getDouble(0.0));
    m_subsystem.resetPose();
    m_subsystem.resetGyroRotation();
    Pose2d transform = new Pose2d(0, -2.4, new Rotation2d());
    m_subsystem.changeRobotPosition(transform);
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
    return false;
  }
}
