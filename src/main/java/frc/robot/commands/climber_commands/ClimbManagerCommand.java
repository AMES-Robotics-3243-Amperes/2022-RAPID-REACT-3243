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
    
    // :) increment the climber steps when the b button is pressed
    if (joystick.getBButton() && !joystick.getXButton() && !m_ClimberSubsystem.isRunningClimbCommand && m_ClimberSubsystem.isCalibrated) { // :) only if the claws are calibrated, there is not command running and only the intended button is pressed
      if (m_ClimberSubsystem.currentClimberStep >= 0 && m_ClimberSubsystem.currentClimberStep < 9 && !m_ClimberSubsystem.isClimberStepStopped) { // :) only goes forward up to step 9, and as long as the current command step wasn't interrupted
        m_ClimberSubsystem.currentClimberStep += 1;
      } else if (m_ClimberSubsystem.currentClimberStep < 0) { // :) in case it wasn't already going forward, redo the undid step
        m_ClimberSubsystem.currentClimberStep *= -1;
        m_ClimberSubsystem.isClimberStepStopped = false;
      } else if (m_ClimberSubsystem.isClimberStepStopped){
        m_ClimberSubsystem.isClimberStepStopped = false;
      }
    }

    // :) increment the climber reverse steps when the x button is pressed
    if (joystick.getXButton() && !joystick.getBButton() && !m_ClimberSubsystem.isRunningClimbCommand && m_ClimberSubsystem.isCalibrated   // :) only if the claws are calibrated, there is not command running and only the intended button is pressed
        && ( (m_ClimberSubsystem.currentClimberStep!=3||m_ClimberSubsystem.currentClimberStep!=-4) && m_ClimberSubsystem.avgSpinnerCurrentDraw>13)) { // :) and the climber is not on the step where it lifted itself up for the first time (as long as it succeeds)
      if (m_ClimberSubsystem.currentClimberStep < 0 && !m_ClimberSubsystem.isClimberStepStopped) { // :) only go reverse until it reaches 0 or it'll start doing positive stuff
        m_ClimberSubsystem.currentClimberStep += 1;
      } else if (m_ClimberSubsystem.currentClimberStep > 0) { // :) if it wasn't already going in reverse, then first undo the current step
        m_ClimberSubsystem.currentClimberStep *= -1;
        m_ClimberSubsystem.isClimberStepStopped = false;
      } else if (m_ClimberSubsystem.isClimberStepStopped){ // :) only resume from the current step if it was stopped in the middle, don't increment
        m_ClimberSubsystem.isClimberStepStopped = false;
      }
    }

    // :) make sure gripper B starts out open and robot is slightly behind first bar

    // :) negative is backwards direction
    // :') I forgot what this comment means^ so ignore it I guess

    if (!m_ClimberSubsystem.isRunningClimbCommand && // :) won't start if a command is already running
        (m_ClimberSubsystem.currentClimberStep!=m_ClimberSubsystem.previousClimberStep || m_ClimberSubsystem.isClimberStepStopped!=m_ClimberSubsystem.prevStopped) && !m_ClimberSubsystem.isClimberStepStopped && // :) won't start a new climb command if you canceled the last one, until it is told which direction to go next
        (DriverStation.isTeleop() && DriverStation.getMatchTime()>1.5)) { // :) won't start a new climb command if match is almost over
      
      // :) this switch statement is basically the climb sequence. They count up from 1 to 9 step by step and the negative versions undo the positive versions (so you can go backwards)

      switch (m_ClimberSubsystem.currentClimberStep) {
        
        case 1:
          // :) first step: it reaches the climb arm up, and opend both grabbers
          new SpinClimberCommand(joystick, m_ClimberSubsystem, 39.85).schedule();
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case -1:
          // :) reverse first step, except it doesn't close the grippers (INTENTIONAL)
          new SpinClimberCommand(joystick, m_ClimberSubsystem, 0).schedule();
          break;
        case 2:
          // :) second step: closes first set of grippers (side 0) on the first bar excluding the spinning one
          new CloseGripperCommand(joystick, m_ClimberSubsystem,Constants.Climber.grabberSide0).schedule(); // :) I dunno what side we should actually actuate first so we'll have to figure it out // :') ok I figured it out it's side 0
          break;
        case -2:
          // :) reverse second step
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case 3:
          // :) third step: rotates the climber arm down so that it lifts itself up, and grabbers on side 1 line up with next bar
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle-77.01).schedule();
          break;
        case -3:
          // :) reverse third step
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle+77.01).schedule();
          break;
        case 4:
          // :) fourth step: closes side 1 grabbers on the 2nd bar
          new CloseGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case -4:
          // :) reverse fourth step
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case 5:
          // :) fifth step: rotates robot towards the next bar, in order to shift center of mass and minimize swinging
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle+28.47).schedule();
          break;
        case -5:
          // :) reverse fifth step
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle-28.47).schedule();
          break;
        case 6:
          // :) sixth step: opens the grippers (side 0) that were closed on the first bars
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case -6:
          // :) reverse sixth step
          new CloseGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case 7:
          // :) seventh step: rotate the bar down and around so that the side 0 grippers now line up with the final bar
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle-137.01).schedule();
          break;
        case -7:
          // :) reverse seventh step
          new SpinClimberCommand(joystick, m_ClimberSubsystem, m_ClimberSubsystem.climberAngle+137.01).schedule();
          break;
        case 8:
          // :) eighth step: close side 0 grabbers onto the last bar
          new CloseGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case -8:
          // :) reverse eighth step and open pawls (because they engage in the subsystem when it sees the grippers closing)
          m_ClimberSubsystem.pawlServoAngles[0]=Constants.Climber.pawlOpen;
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
          break;
        case 9:
          // :) ninth step: release the side 1 grabbers, completing the climb. this will cause the robot to swing since it doesn't readjust its center of mass, but it's fine
          new OpenGripperCommand(joystick, m_ClimberSubsystem, Constants.Climber.grabberSide1).schedule();
          break;
        case -9:
          // :) DO NOTHING!!! don't reverse this step and also don't allow the robot to reverse after it reaches step 9
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
