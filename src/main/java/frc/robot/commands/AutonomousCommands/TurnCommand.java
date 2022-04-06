// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final double m_radians;
  private final Timer timer;
  /** Creates a new TurnCommand. */
  public TurnCommand(DriveSubsystem subsystem, double radians) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_radians = radians;
    addRequirements(m_subsystem);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.changeRobotAngle(m_radians);
    timer.reset();
    timer.start();
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
    return m_subsystem.atTargetPosition() || (timer.get() > Constants.DriveTrain.autoTimeout);
  }
}
