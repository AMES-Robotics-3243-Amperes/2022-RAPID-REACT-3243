// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// ++ comment so I can merge into main

//  ++ FRC stuff
package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// ++ project stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// ++ misc
import frc.robot.subsystems.IMUSubsystem;

// ++ SUBSYSTEMS
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeIndexerSubsystem;


// ++ COMMANDS
  // ++ teleop
import frc.robot.commands.TeleopPIDDriveCommand;
import frc.robot.commands.IntakeIndexer.AcceptCommand;
import frc.robot.commands.IntakeIndexer.RebuffCommand;
import frc.robot.commands.IntakeIndexer.SpinTakeCommand;
import frc.robot.commands.TestPid;
import frc.robot.commands.AutonomousCommands.AutonomousCommand;
import frc.robot.commands.AutonomousCommands.AutonomousTaxiCommand;
import frc.robot.commands.ShooterStuff.ShooterCommand;
import frc.robot.commands.ShooterStuff.AutoShooterRoutineCommands.LimelightAlignDriveCommand;
import frc.robot.commands.ShooterStuff.AutoShooterRoutineCommands.ShootRoutineCommandGroup;
import frc.robot.commands.ShooterStuff.DumpShooterCommands.DumpCommandGroup;
// ++ auto
import frc.robot.commands.AutonomousCommands.LookAtCommand;



/* Luke's (CAD lead) helpful code:
* public class object final void robotclassobjectfinal{
*      controller.main.public.alpha.beta.epsilon.epislon.axes.axises.axis1.speed.speedValue =loudTyping.lound.loud.thing.class.private.public.static.dynamic.void.int.double.float.variableType.one.two.three.finalXYZ;
*
*}
*/

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.climber_commands.ClimbManagerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ++ comment so I can rebase

// ++ = max
// :) = mason

//make the climb able to go back if it gets stuck
// also make sure user can't open gripper when lock engaged
//grabber locking: 180 degrees is locking, 150 is unlocked or not I guess zain got that wrong
//todo for grabber lock: in the case of an emergency and as a failsafe, if the robot detects something go wrong (motor controller disconnected, motor encoder reading dangerous values, etc) lock engages automatically and climber ability disables
//ok actually sounds like it's gonna be better if we readjust the COM before/during letting go of the last bar

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // :) this is a timer object, using in climber
  public static DriverStation m_driverStation;

  // ++ JOYSTICK STUFF ========================================
  // ++ we make a JoyUtil object instead of an XboxController object; JoyUtil inherits XboxController
  public static JoyUtil primaryController = new JoyUtil( Constants.Joysticks.primaryControllerID );
  public static JoyUtil secondaryController = new JoyUtil( Constants.Joysticks.secondaryControllerID );
    public static JoystickButton secondaryRightBumper = new JoystickButton(secondaryController, Constants.Joysticks.RightBumper);
    public static JoystickButton secondaryXButton = new JoystickButton(secondaryController, Constants.Joysticks.X);
    public static JoystickButton primaryAButton = new JoystickButton(primaryController, Constants.Joysticks.A);
    public static JoystickButton autoShootRoutineButton = new JoystickButton(secondaryController, Constants.Joysticks.Y);
    public static JoystickButton dumpButton = new JoystickButton(secondaryController, Constants.Joysticks.B);



  // ++ SUBSYSTEMS AND COMMANDS ========================================
  // SUBSYSTEMS -------------------
    // ++ "utility subsystems"
  private final IMUSubsystem m_IMUSubsystem = new IMUSubsystem();
  private final ShuffleboardSubsystem m_Shuffleboardsubsystem = new ShuffleboardSubsystem();

    // ++ robot subsystems
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  // commands
  private final ClimbManagerCommand m_ClimbCommand = new ClimbManagerCommand(m_ClimberSubsystem, secondaryController);
  // ++ =================================================
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final IntakeIndexerSubsystem m_IntakeIndexerSubsystem = new IntakeIndexerSubsystem();
  private final HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
  // COMMANDS--------------------
    // ++ teleop commands
  private final TeleopPIDDriveCommand m_PIDDriveCommand = new TeleopPIDDriveCommand(m_DriveSubsystem, primaryController);
  private final ShooterCommand m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem, secondaryController);
  // ++ m_AcceptCommand probably shouldn't be used in competition, but idk
  private final AcceptCommand m_AcceptCommand = new AcceptCommand(m_IntakeIndexerSubsystem, false, Constants.IntakeIndexer.acceptDuration);
  private final RebuffCommand m_RebuffCommand = new RebuffCommand(m_IntakeIndexerSubsystem);
  private final SpinTakeCommand m_SpinIntakeCommand = new SpinTakeCommand(m_IntakeIndexerSubsystem, secondaryController);
  private final AutonomousCommand m_AutonomousCommand = new AutonomousCommand(m_DriveSubsystem, m_ShooterSubsystem, m_IntakeIndexerSubsystem);
  private final LookAtCommand m_lookAtCommand = new LookAtCommand(m_DriveSubsystem, true);

  private final ShootRoutineCommandGroup m_AutoShootRoutineCommand = new ShootRoutineCommandGroup(
    m_DriveSubsystem,
    m_IntakeIndexerSubsystem,
    m_ShooterSubsystem,
    m_HoodSubsystem
    );

  private final DumpCommandGroup m_DumpCommand = new DumpCommandGroup(m_ShooterSubsystem, m_IntakeIndexerSubsystem, m_HoodSubsystem);

    // ++ auto commands


  // ++ END SUBSYSTEMS/COMMANDS ===============================================


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // ++ command stuff
    // m_DriveSubsystem.setDefaultCommand(m_DriveCommand);
    m_ClimberSubsystem.setDefaultCommand(m_ClimbCommand);
    m_DriveSubsystem.setDefaultCommand(m_PIDDriveCommand);
    m_ShooterSubsystem.setDefaultCommand(m_ShooterCommand);
    m_IntakeIndexerSubsystem.setDefaultCommand(m_SpinIntakeCommand);
    configureButtonBindings();



  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    secondaryRightBumper.whenPressed(m_RebuffCommand);
    secondaryXButton.whenPressed(m_AcceptCommand);
    autoShootRoutineButton.whenPressed(m_AutoShootRoutineCommand);
    dumpButton.whenPressed(m_DumpCommand);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return null;
    // return new TestPid(m_DriveSubsystem);
    return m_lookAtCommand;
  }

  public ClimberSubsystem getClimberSubsystem(){
    return m_ClimberSubsystem;
  }

}
