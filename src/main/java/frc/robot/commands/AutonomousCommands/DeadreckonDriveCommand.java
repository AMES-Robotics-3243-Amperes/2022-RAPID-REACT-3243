// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DeadreckonDriveCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private final MecanumDriveWheelSpeeds m_speeds;
  private final double m_duration;
  private final Timer timer;
  /** Creates a new DeadreckenDriveCommand. */
  public DeadreckonDriveCommand(DriveSubsystem subsystem, MecanumDriveWheelSpeeds speeds, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    m_speeds = speeds;
    m_duration = duration;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setVelocityReference(m_speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= m_duration;
  }
}
