// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SpinClimberCommand extends CommandBase {
  private static ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  public double goalRevolution;
  
  
  /** Creates a new SpinClimberCommand. */
  public SpinClimberCommand(ClimberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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
    if (Math.abs(goalRevolution-m_ClimberSubsystem.encoderClimberAngle)<0.05) { // error room for end command
      return true;
    } else {
      return false;
    }
  }
}
