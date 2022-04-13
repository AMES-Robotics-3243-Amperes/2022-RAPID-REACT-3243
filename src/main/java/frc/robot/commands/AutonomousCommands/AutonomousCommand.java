// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterStuff.ShooterCommand;
import frc.robot.commands.ShooterStuff.AutoShooterRoutineCommands.ShootRoutineCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeIndexer.AutonomousSpintakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  // DriveSubsystem drive;
  // ShooterSubsystem m_shooter;
  // IntakeIndexerSubsystem m_intake;
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand(DriveSubsystem drive, ShooterSubsystem shooter, IntakeIndexerSubsystem intake, HoodSubsystem hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // drive = drive;
    // m_shooter = shooter;
    // m_intake = intake;
    // addRequirements(m_drive, m_shooter, m_intake);
    // InstantCommand initializeAutonomous = new InstantCommand(drive::toAutonomousMode, drive);
    // AutonomousTaxiCommand moveToBall = new AutonomousTaxiCommand(drive, true);
    // AutonomousSpintakeCommand intakeBall = new AutonomousSpintakeCommand(intake, drive);
    // LookAtCommand turnToHub = new LookAtCommand(drive, false);
    // InstantCommand initializeTeleop = new InstantCommand(drive::toTeleopMode, drive);
    MecanumDriveWheelSpeeds forwardSpeeds =  new MecanumDriveWheelSpeeds(Constants.DriveTrain.maxWheelSpeed/-5, Constants.DriveTrain.maxWheelSpeed/-5, Constants.DriveTrain.maxWheelSpeed/-5, Constants.DriveTrain.maxWheelSpeed/-5);
    addCommands(new SetPoseCommand(drive),
                new GoToCommand(drive, true),
                new InstantCommand(drive::stopRobot, drive),
                new WaitCommand(0.75),
                new ParallelCommandGroup(new AutonomousSpintakeCommand(intake, 2), new DeadreckonDriveCommand(drive, forwardSpeeds, 2)),
                // new GoToCommand(drive, false);


                // More cursed coefficients woo! This makes it do a 180, don't ask why this many radians is a U-turn to the robot
                new ParallelCommandGroup(new TurnCommand(drive, (1.275 * Math.PI)), new AutonomousSpintakeCommand(intake, 2)),
                new ShootRoutineCommandGroup(drive, intake, shooter, hood, null),
                new InstantCommand(drive::resetOutputRange, drive),
                new InstantCommand(drive::stopRobot, drive));
  }
}