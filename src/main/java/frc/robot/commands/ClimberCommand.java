// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.JoyUtil;
import frc.robot.Constants; 

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ClimberCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_ClimberSubsystem;
  private final JoyUtil joystick;

  public boolean climberMode = true;

  //climber speeds
  private double half_1;
  private double half_2;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCommand(ClimberSubsystem subsystem, JoyUtil secondaryController) {
    m_ClimberSubsystem = subsystem;
    joystick = secondaryController; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // :) gets left and right trigger axes to spin the shaktool climber
    // :) they cancel each other out as well
    half_1 = joystick.getRightTriggerAxis()/2;
    half_2 = joystick.getLeftTriggerAxis()/-2;
    m_ClimberSubsystem.spinClimber(half_1+half_2);

    if (joystick.getXButton() && !joystick.getYButton()) {
      m_ClimberSubsystem.spinGrabbers(0, 0.2);
    } else if (joystick.getYButton()) {
      m_ClimberSubsystem.spinGrabbers(0, -0.2);
    }

    if (joystick.getAButton() && !joystick.getBButton()) {
      m_ClimberSubsystem.spinGrabbers(1, 0.2);
    } else if (joystick.getBButton()) {
      m_ClimberSubsystem.spinGrabbers(1, -0.2);
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
