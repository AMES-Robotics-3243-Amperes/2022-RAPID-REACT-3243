// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.climber_commands.CloseGripperCommand;
import frc.robot.commands.climber_commands.OpenGripperCommand;
import frc.robot.commands.climber_commands.SpinClimberCommand;
import frc.robot.commands.climber_commands.GeneralClimbCommand;
import frc.robot.JoyUtil;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbManagerCommand extends CommandBase {
  private JoyUtil joystick;
  private ClimberSubsystem m_ClimberSubsystem;


  /** Creates a new ClimbStepManageCommand. */
  public ClimbManagerCommand(ClimberSubsystem subsystem, JoyUtil joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ClimberSubsystem = subsystem;
    addRequirements(m_ClimberSubsystem);
    joystick=joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //command process for climbing, starts with robot lined up inside 2nd bar (of 4), intake facing away from driver stations, and nearest grabber clamped on 2nd bar
    if (joystick.getBButton() && joystick.getXButton() == false && m_ClimberSubsystem.isRunningClimbCommand == false) {
      if (m_ClimberSubsystem.currentClimberStep > 0) {
        m_ClimberSubsystem.currentClimberStep += 1;
      } else if (m_ClimberSubsystem.currentClimberStep < 0) {
        m_ClimberSubsystem.currentClimberStep *= -1;
      }
    }

    if (joystick.getXButton() && joystick.getBButton() == false && m_ClimberSubsystem.isRunningClimbCommand == false) {
      if (m_ClimberSubsystem.currentClimberStep < 0) {
        m_ClimberSubsystem.currentClimberStep -= 1;
      } else if (m_ClimberSubsystem.currentClimberStep > 0) {
        m_ClimberSubsystem.currentClimberStep *= -1;
      }
    }

    // negative is backwards direction
    if (m_ClimberSubsystem.isRunningClimbCommand == false) {
      switch (m_ClimberSubsystem.currentClimberStep) {
        
      case 1:
        //rotate arm(~120)
        
      case 2:
        //close gripper(B)

      case 3:
        //rotate arm(~-90)

      case 4:
        //open gripper(A)

      case 5:
        //rotate arm(~180)

      case 6:
        //close gripper(A)

      case 7:
        //rotate arm(~-90) -optional

      case 8:
        //open gripper(B)
      
      case -1:
        //rotate arm(~-120)
        
      case -2:
        //open gripper(B)

      case -3:
        //rotate arm(~90)

      case -4:
        //close gripper(A)

      case -5:
        //rotate arm(~-180)

      case -6:
        //open gripper(A)
      
      // -7 and -8 are probably unnecessary
      case -7:
        //rotate arm(~90) -optional

      case -8:
        //close gripper(B)

      }
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
