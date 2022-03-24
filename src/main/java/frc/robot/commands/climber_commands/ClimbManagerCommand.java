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

public class ClimbManagerCommand extends CommandBase {
  private JoyUtil joystick;
  private ClimberSubsystem m_ClimberSubsystem;
  public static SpinClimberCommand m_SpinClimberCommand;
  public static ManualClimbCommand m_ManualClimbCommand;


  /** Creates a new ClimbStepManageCommand. */
  public ClimbManagerCommand(ClimberSubsystem subsystem, JoyUtil secondaryController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    m_SpinClimberCommand = new SpinClimberCommand(subsystem);
    m_ManualClimbCommand = new ManualClimbCommand(subsystem, secondaryController);
    addRequirements(m_ClimberSubsystem);
    joystick=secondaryController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ManualClimbCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // :) command process for climbing, starts with robot lined up inside 2nd bar (of 4), intake facing away from driver stations, and nearest grabber clamped on 2nd bar
    if (joystick.getBButton() && joystick.getXButton() == false && m_ClimberSubsystem.isRunningClimbCommand == false) {
      if (m_ClimberSubsystem.currentClimberStep > 0) {
        m_ClimberSubsystem.currentClimberStep += 1;
      } else if (m_ClimberSubsystem.currentClimberStep < 0) {
        m_ClimberSubsystem.currentClimberStep *= -1;
      }
    }

    if (joystick.getXButton() && joystick.getBButton() == false && m_ClimberSubsystem.isRunningClimbCommand == false) {
      if (m_ClimberSubsystem.currentClimberStep < 0) {
        m_ClimberSubsystem.currentClimberStep += 1;
      } else if (m_ClimberSubsystem.currentClimberStep > 0) {
        m_ClimberSubsystem.currentClimberStep *= -1;
      }
    }

    // :) make sure gripper B starts out open
    // :) negative is backwards direction
    if (m_ClimberSubsystem.isRunningClimbCommand == false && m_ClimberSubsystem.currentClimberStep!=m_ClimberSubsystem.previousClimberStep) {
      switch (m_ClimberSubsystem.currentClimberStep) {
        
        case 1:
          new CloseGripperCommand(m_ClimberSubsystem,Constants.Climber.grabberSide0).schedule(); // :) I dunno what side we should actually actuate first so we'll have to figure it out
        case -1:
          //open gripper(A)
          new OpenGripperCommand(m_ClimberSubsystem, Constants.Climber.grabberSide0).schedule();
        case 2:
          // rotate arm(~90)
          // m_SpinClimberCommand.goalRevolution = m_ClimberSubsystem.encoderClimberAngle
          // + ?;
          // m_SpinClimberCommand.schedule();
        case -2:
          // rotate arm(~-120)

        case 3:
          // close gripper(B)

        case -3:
          // open gripper(B)

        case 4:
          // rotate arm(~-90) probably closer to -75 degrees

        case -4:
          // rotate arm(~90) probably closer to 75 degrees

        case 5:
          // open gripper(A)

        case -5:
          // close gripper(A)

        case 6:
          // rotate arm(~180) probably closer to 240 degrees

        case -6:
          // rotate arm(~-180) probably closer to -240 degrees

        case 7:
          // close gripper(A)

        case -7:
          // open gripper(A)

        case 8:
          // rotate arm(~-90) -optional, probably closer to -75 degrees

        case -8:
          // rotate arm(~90) -optional, probably closer to 75 degrees

        case 9:
          // open gripper(B)

        case -9:
          // close gripper(B)

      // -7 and -8 are probably unnecessary
      
      
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
