// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.climber_commands.CloseGripperCommand;
import frc.robot.commands.climber_commands.OpenGripperCommand;
import frc.robot.commands.climber_commands.SpinClimberCommand;
import frc.robot.commands.climber_commands.GeneralClimbCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbManagerCommand extends CommandBase {
  ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();


  /** Creates a new ClimbStepManageCommand. */
  public ClimbManagerCommand(ClimberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (m_ClimberSubsystem.currentClimberStep) {

    case 1:


    case 2:


    case 3:


    case 4:


    case 5:

    
    }
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
