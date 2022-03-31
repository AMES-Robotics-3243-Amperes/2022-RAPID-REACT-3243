// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.climber_commands.CloseGripperCommand;
import frc.robot.commands.climber_commands.OpenGripperCommand;
import frc.robot.commands.climber_commands.SpinClimberCommand;
import frc.robot.commands.climber_commands.ManualClimbCommand;
import frc.robot.Constants;
import frc.robot.JoyUtil;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

public class ClimbManagerCommand extends CommandBase {
  private JoyUtil joystick;
  private ClimberSubsystem m_ClimberSubsystem;
  public static SpinClimberCommand m_SpinClimberCommand;
  public static ManualClimbCommand m_ManualClimbCommand;


  /** Creates a new ClimbStepManageCommand. */
  public ClimbManagerCommand(ClimberSubsystem subsystem, JoyUtil secondaryController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    m_ManualClimbCommand = new ManualClimbCommand(subsystem, secondaryController);
    addRequirements(m_ClimberSubsystem);
    joystick=secondaryController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ManualClimbCommand.schedule();
    m_ClimberSubsystem.isRunningClimbCommand = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // :) command process for climbing, starts with robot lined up inside 2nd bar (of 4), intake facing away from driver stations, and nearest grabber clamped on 2nd bar
    if (joystick.getBButton() && joystick.getXButton() == false && m_ClimberSubsystem.isRunningClimbCommand == false && m_ClimberSubsystem.isCalibrated) {
      if (m_ClimberSubsystem.currentClimberStep >= 0 && m_ClimberSubsystem.currentClimberStep < 9 && m_ClimberSubsystem.isClimberStepStopped==false) {
        m_ClimberSubsystem.currentClimberStep += 1;
      } else if (m_ClimberSubsystem.currentClimberStep < 0) {
        m_ClimberSubsystem.currentClimberStep *= -1;
        m_ClimberSubsystem.isClimberStepStopped = false;
      } else if (m_ClimberSubsystem.isClimberStepStopped){
        m_ClimberSubsystem.isClimberStepStopped = false;
      }
    }

    if (joystick.getXButton() && joystick.getBButton() == false && m_ClimberSubsystem.isRunningClimbCommand == false && m_ClimberSubsystem.isCalibrated
        && ( (m_ClimberSubsystem.currentClimberStep!=3||m_ClimberSubsystem.currentClimberStep!=-4) && m_ClimberSubsystem.avgSpinnerCurrentDraw>13)) {
      if (m_ClimberSubsystem.currentClimberStep < 0) {
        m_ClimberSubsystem.currentClimberStep += 1;
      } else if (m_ClimberSubsystem.currentClimberStep > 0) {
        m_ClimberSubsystem.currentClimberStep *= -1;
        m_ClimberSubsystem.isClimberStepStopped = false;
      } else if (m_ClimberSubsystem.isClimberStepStopped){
        m_ClimberSubsystem.isClimberStepStopped = false;
      }
    }

    // :) make sure gripper B starts out open
    // :) negative is backwards direction
    if (m_ClimberSubsystem.isRunningClimbCommand == false &&
        (m_ClimberSubsystem.currentClimberStep!=m_ClimberSubsystem.previousClimberStep || m_ClimberSubsystem.isClimberStepStopped!=m_ClimberSubsystem.prevStopped) && m_ClimberSubsystem.isClimberStepStopped==false &&
        (DriverStation.isTeleop() && DriverStation.getMatchTime()>1.5)) {
      
      switch (m_ClimberSubsystem.currentClimberStep) {
        
        case 1:
          new SpinClimberCommand(joystick, m_ClimberSubsystem, 39.85).schedule();
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case -1:
          new SpinClimberCommand(joystick, m_ClimberSubsystem, 0).schedule();
          //new CloseGripperCommand(m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          //new CloseGripperCommand(m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case 2:
          new CloseGripperCommand(joystick, m_ClimberSubsystem,Constants.Climber.grabberSide0).schedule(); // :) I dunno what side we should actually actuate first so we'll have to figure it out
          break;
        case -2:
          //open gripper(A)
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case 3:
          // rotate arm(~90)
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle-77.01).schedule(); // this should be -77.01
          break;
        case -3:
          // rotate arm(~-90)
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle+77.01).schedule(); // this should be +77.01
          break;
        case 4:
          // close gripper(B)
          new CloseGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case -4:
          // open gripper(B)
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case 5:
          // rotate arm(~-90) probably closer to -75 degrees
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle+28.47).schedule();
          break;
        case -5:
          // rotate arm(~90) probably closer to 75 degrees
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle-28.47).schedule();
          break;
        case 6:
          // open gripper(A)
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case -6:
          // close gripper(A)
          new CloseGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case 7:
          // rotate arm(~180) probably closer to 240 degrees
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle-137.01).schedule();
          break;
        case -7:
          // rotate arm(~-180) probably closer to -240 degrees
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle+137.01).schedule();
          break;
        case 8:
          // close gripper(A)
          new CloseGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case -8:
          // open gripper(A)
          m_ClimberSubsystem.pawlServoAngles[0]=Constants.Climber.pawlOpen;
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case 9:
          // open gripper(B)
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case -9:
          // DO NOTHING!!!
          //new CloseGripperCommand(m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        default:

          break;
      
      
      }
    }
    m_ClimberSubsystem.previousClimberStep = m_ClimberSubsystem.currentClimberStep;
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
