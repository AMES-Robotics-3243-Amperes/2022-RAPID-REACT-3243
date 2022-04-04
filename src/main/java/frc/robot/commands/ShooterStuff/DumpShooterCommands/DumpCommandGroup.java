// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterStuff.DumpShooterCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeIndexer.AcceptCommand;
import frc.robot.commands.ShooterStuff.StopFlywheelCommand;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** This is the command group for dumping the balls into the low hub */
public class DumpCommandGroup extends SequentialCommandGroup{

    public DumpCommandGroup(
        ShooterSubsystem m_ShooterSubsystem,
        IntakeIndexerSubsystem m_IntakeIndexerSubsystem
    ) {

        addCommands(

            new DumpFlywheelCommand(m_ShooterSubsystem),
            new AcceptCommand( m_IntakeIndexerSubsystem, false),
            new AcceptCommand( m_IntakeIndexerSubsystem, false),
            new StopFlywheelCommand( m_ShooterSubsystem)
            
        );

    }



}
