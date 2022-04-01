// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutnomousCommands.GoToCommand;
import frc.robot.commands.AutnomousCommands.LookAtCommand;
import frc.robot.commands.AutnomousCommands.SetPoseCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTaxiCommand extends SequentialCommandGroup {
  /** Creates a new AutonomousTaxiCommand. */
  public AutonomousTaxiCommand(DriveSubsystem subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetPoseCommand(subsystem), new LookAtCommand(subsystem), new GoToCommand(subsystem));
  }

}
