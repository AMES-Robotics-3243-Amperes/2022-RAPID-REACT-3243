// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.JoyUtil;
import frc.robot.Constants; 

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_DriveSubsystem;
  private final XboxController joystick;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem, XboxController driveController) {
    m_DriveSubsystem = subsystem;
    joystick = driveController; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ++ we want to curve the x and y speeds the same, but we'll probably do the rotation differently
    double speedX = JoyUtil.joyCurve( JoyUtil.posWithDeadzone( joystick.getLeftX()));
    double speedY = JoyUtil.joyCurve( JoyUtil.posWithDeadzone( joystick.getLeftY()));

    double speedR = Constants.Joysticks.rotationDamper * JoyUtil.posWithDeadzone( joystick.getRightX()); 
    
    double speedMultiplier = Constants.DriveTrain.totalSpeedDamper; 
    m_DriveSubsystem.setMotors(speedX * speedMultiplier, speedY * speedMultiplier, speedR * speedMultiplier); 

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
