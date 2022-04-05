// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeIndexer.AutonomousSpintakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  IntakeIndexerSubsystem m_intake;
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(DriveSubsystem drive, ShooterSubsystem shooter, IntakeIndexerSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drive = drive;
    m_shooter = shooter;
    m_intake = intake;
    addRequirements(m_drive, m_shooter, m_intake);
    InstantCommand initializeAutonomous = new InstantCommand(m_drive::toAutonomousMode, m_drive);
    AutonomousTaxiCommand moveToBall = new AutonomousTaxiCommand(m_drive, true);
    AutonomousSpintakeCommand intakeBall = new AutonomousSpintakeCommand(m_intake, m_drive);
    LookAtCommand turnToHub = new LookAtCommand(m_drive, false);
    InstantCommand initializeTeleop = new InstantCommand(m_drive::toTeleopMode, m_drive);
    addCommands(initializeAutonomous, moveToBall, intakeBall, turnToHub, initializeTeleop);
  }
}