// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import java.io.Console;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

public class GoToCommand extends CommandBase {

  private final DriveSubsystem m_subsystem;
  private Pose2d m_target;
  private final boolean m_toBall;
  private final Timer timer;
  /** Creates a new GoToCommand. */
  public GoToCommand(DriveSubsystem subsystem, boolean toBall) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    m_toBall = toBall;
    addRequirements(m_subsystem);
    timer = new Timer();
  }
  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {
    m_subsystem.setPIDValues(0.6, 0, 0);
    if (m_toBall) {
      m_target = ShuffleboardSubsystem.getTargetPose();
    }else {
      m_target = ShuffleboardSubsystem.getStartingPose();
    }
    m_subsystem.setRobotPosition(m_target);
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
    return m_subsystem.atTargetPosition() || (timer.get() >= Constants.DriveTrain.autoTimeout) ;
  }
}
