// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.JoyUtil;

// :) this command's action really only needs to be run once, and in all honesty it probably doesn't really need to be a command, but I think it's cleaner this way

public class SpinClimberCommand extends CommandBase {
  private static ClimberSubsystem m_ClimberSubsystem;
  private static JoyUtil joystick;
  public double goalRevolution;
  
  
  /** Creates a new SpinClimberCommand. */
  public SpinClimberCommand(JoyUtil joy, ClimberSubsystem subsystem, double revolutions) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
    goalRevolution = revolutions; //*(230.4/360.0)
    joystick = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ClimberSubsystem.isRunningClimbCommand = true;
    // :) yup this tells the climber to rotate to a position
    m_ClimberSubsystem.actuateClimber(goalRevolution);

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
    // :) stops the command if the position gets achieved successfully or if it was interrupted by pushing the pause button (start)
    if (Math.abs(goalRevolution-m_ClimberSubsystem.encoderClimberAngle)<2.5 || (joystick.getStartButton())) { // error room for end command
      if ( (joystick.getStartButton())){
        m_ClimberSubsystem.grabberHoldAngles = m_ClimberSubsystem.encoderGrabberAngles;
        m_ClimberSubsystem.climberHoldAngle = m_ClimberSubsystem.encoderClimberAngle;
        m_ClimberSubsystem.isClimberStepStopped = true;
      }
      m_ClimberSubsystem.isRunningClimbCommand = false;
      return true;
    } else {
      return false;
    }
  }
}
