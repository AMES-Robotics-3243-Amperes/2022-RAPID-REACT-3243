// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class recalibrate_grippers extends CommandBase {
  ClimberSubsystem m_ClimberSubsystem;
  /** Creates a new recalibrate_grippers. */
  public recalibrate_grippers(ClimberSubsystem subsystem) {
    addRequirements(subsystem);
    m_ClimberSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_ClimberSubsystem.currentClimberStep == 0){
      m_ClimberSubsystem.calibrateGrabbers();
    }
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
