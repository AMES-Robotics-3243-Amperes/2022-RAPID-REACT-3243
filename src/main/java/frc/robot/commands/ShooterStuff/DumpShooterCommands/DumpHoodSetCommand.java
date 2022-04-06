// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterStuff.DumpShooterCommands;

import frc.robot.subsystems.HoodSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.LimelightSubsystem;


public class DumpHoodSetCommand extends CommandBase {

  private final HoodSubsystem m_HoodSubsystem;

  private final Timer clock;

  /** Creates a new DumpHoodSetCommand. */
  public DumpHoodSetCommand(HoodSubsystem hoodSubsystem) {
    m_HoodSubsystem = hoodSubsystem;
    clock = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_HoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clock.reset();
    clock.start();
    m_HoodSubsystem.setServoPositionFromHoodAngle( Constants.Shooter.dumpHoodAngle );
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
    return (clock.get() >= Constants.Shooter.dumpHoodTimeout);
  }
}
