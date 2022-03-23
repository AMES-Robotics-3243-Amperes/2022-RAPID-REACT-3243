// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class OpenGripperCommand extends CommandBase {
  private static ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  public int actuatingSide = -1;

  
  /** Creates a new CloseGripperCommand. */
  public OpenGripperCommand(ClimberSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (actuatingSide != -1){
      m_ClimberSubsystem.actuateGrabber(actuatingSide, m_ClimberSubsystem.gripperOpenMaximum);
    } else {
      System.out.println("Uh... something went wrong in the gripper open command. You're not setting the actuation side!");
    }
  }

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
    if (m_ClimberSubsystem.encoderGrabberAngles[actuatingSide] > m_ClimberSubsystem.gripperOpenMaximum-0.05) { //0.05 is the error room to stop the function.
      return true;
    } else {
      return false;
    }
  }
}
