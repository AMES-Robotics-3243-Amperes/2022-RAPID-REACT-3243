// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeIndexerSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RebuffCommand;
import frc.robot.commands.AcceptCommand;
import frc.robot.commands.SpinIntakeCommand;

import frc.robot.Constants;
import frc.robot.Constants.Joysticks; 


// ++ comment so I can rebase

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  // ++ SUBSYSTEMS AND COMMANDS ========================================
  // subsystems
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final IntakeIndexerSubsystem m_IntakeIndexerSubsystem = new IntakeIndexerSubsystem();
  // commands
  private final DriveCommand m_DriveCommand = new DriveCommand(m_DriveSubsystem, primaryController);
  private final AcceptCommand m_AcceptCommand = new AcceptCommand(m_IntakeIndexerSubsystem, Constants.IntakeIndexer.acceptRotations);
  private final RebuffCommand m_RebuffCommand = new RebuffCommand(m_IntakeIndexerSubsystem, Constants.IntakeIndexer.rebuffRotations, Constants.IntakeIndexer.rebuffSpeed, Constants.IntakeIndexer.rebuffDuration);
  private final SpinIntakeCommand m_SpinIntakeCommand = new SpinIntakeCommand(m_IntakeIndexerSubsystem, primaryController);
  // ++ =================================================



  // ++ JOYSTICK STUFF ========================================
  public static XboxController primaryController = new XboxController( Constants.Joysticks.primaryControllerID );
  public static XboxController secondaryController = new XboxController( Constants.Joysticks.secondaryControllerID );
  public static JoystickButton pb_rightBumper = new JoystickButton(primaryController, Constants.Joysticks.RightBumper);
  public static JoystickButton pb_leftBumper = new JoystickButton(primaryController, Constants.Joysticks.LeftBumper);
    
  



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // ++ command stuff
    m_DriveSubsystem.setDefaultCommand(m_DriveCommand);
    m_IntakeIndexerSubsystem.setDefaultCommand(m_SpinIntakeCommand);


    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    pb_leftBumper.whenPressed(m_RebuffCommand);
    pb_rightBumper.whenPressed(m_AcceptCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null; 
  }
}
