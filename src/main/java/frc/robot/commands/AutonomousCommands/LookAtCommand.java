// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

public class LookAtCommand extends CommandBase {
  private final DriveSubsystem m_subsystem;
  private Pose2d m_target;
  private final boolean m_toBall;
  /** Creates a new LookAtCommand using the target position from shuffleboard. */
  public LookAtCommand(DriveSubsystem subsystem, boolean toBall) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
    m_toBall = toBall;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // If m_toBall is true, the robot will point towards the target ball. If not, it will point to it's original starting position
    if (m_toBall) {
      m_target = ShuffleboardSubsystem.getTargetPose();
    }else {
      m_target = ShuffleboardSubsystem.getStartingPose();
    }
    m_subsystem.lookAt(m_target);
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
    return m_subsystem.atTargetPosition();
  }
}
