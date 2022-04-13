// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.JoyUtil;
import edu.wpi.first.wpilibj.DriverStation;

// :) this command's action really only needs to be run once, and in all honesty it probably doesn't really need to be a command, but I think it's cleaner this way

public class OpenGripperCommand extends CommandBase {
  private static JoyUtil joystick;
  private static ClimberSubsystem m_ClimberSubsystem;
  public int actuatingSide = -1;



  /** Creates a new CloseGripperCommand. */
  public OpenGripperCommand(JoyUtil joy, ClimberSubsystem subsystem, int side) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
    actuatingSide = side;
    joystick = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_ClimberSubsystem.isRunningClimbCommand = true;
    if (actuatingSide != -1){
      // :) you guessed it, this opens the grippers
      m_ClimberSubsystem.actuateGrabber(actuatingSide, m_ClimberSubsystem.gripperOpenMaximum);
    } else {
      System.err.println("Uh... something went wrong in the gripper open command. You're somehow not setting the actuation side!");
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
    // :) stops the command if the position gets achieved successfully or if it was interrupted by pushing the pause button
    if (m_ClimberSubsystem.encoderGrabberAngles[actuatingSide] > m_ClimberSubsystem.gripperOpenMaximum-1.5 || joystick.getStartButton() || (DriverStation.getMatchTime()<1.5 && DriverStation.isTeleop())) { //1 is the error room to stop the function.
      if ( joystick.getStartButton()){
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
