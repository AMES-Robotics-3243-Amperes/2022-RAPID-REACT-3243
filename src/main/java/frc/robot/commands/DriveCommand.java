// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.JoyUtil;

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
  public DriveCommand(DriveSubsystem subsystem, XboxController movingstick) {
    joystick = movingstick;
    m_DriveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double multiplier(double axis) {
    return (1+(3 * axis));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x_axis = joystick.getLeftX()/4;
    double y_axis = joystick.getLeftY()/4;
    double z = joystick.getRightX()/4;
    x_axis = JoyUtil.deadzone(x_axis);
    y_axis = JoyUtil.deadzone(y_axis);
    z = JoyUtil.deadzone(z);
    x_axis *= multiplier(joystick.getRightTriggerAxis());
    y_axis *= multiplier(joystick.getRightTriggerAxis());
    z *= multiplier(joystick.getRightTriggerAxis());
    m_DriveSubsystem.driveCartesian(x_axis, y_axis, z);
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
