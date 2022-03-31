// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber_commands;
import frc.robot.JoyUtil;
import frc.robot.Constants; 

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ManualClimbCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_ClimberSubsystem;
  private final JoyUtil joystick;

  public boolean climberMode = true;

  //climber speeds
  private double spinSpeed;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualClimbCommand(ClimberSubsystem subsystem, JoyUtil secondaryController) {
    m_ClimberSubsystem = subsystem;
    joystick = secondaryController; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberSubsystem.calibrateGrabbers();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // :) gets left and right trigger axes to spin the shaktool climber for fine manual adjustments (shouldn't need this if everything goes well)
    // :) they cancel each other out as well

    if (m_ClimberSubsystem.isRunningClimbCommand == false) {
      if (Math.abs(joystick.getRawAxis(0))>0.2) {
        spinSpeed = joystick.getRawAxis(0)/4;
      }
      m_ClimberSubsystem.spinClimber(spinSpeed);
    }

    if (m_ClimberSubsystem.isTooHot){
      joystick.setRumble(RumbleType.kLeftRumble, 1);
      joystick.setRumble(RumbleType.kRightRumble, 1);
    } else {
      joystick.setRumble(RumbleType.kLeftRumble, 0);
      joystick.setRumble(RumbleType.kRightRumble, 0);
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
