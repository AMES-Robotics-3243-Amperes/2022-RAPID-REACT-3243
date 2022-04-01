// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//  ++ FRC stuff
package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// ++ project stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// ++ misc
import frc.robot.subsystems.IMUSubsystem;

// ++ SUBSYSTEMS
import frc.robot.subsystems.DriveSubsystem;
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
  // ++ auto
import frc.robot.commands.AutonomousPIDTaxiCommand;
import frc.robot.commands.ShooterCommand;
  // ++ shooter routine
import frc.robot.commands.ShooterRoutineCommands.LimelightAlignDriveCommand;
import frc.robot.commands.ShooterRoutineCommands.ShootRoutineCommandGroup;



/* Luke's (CAD lead) helpful code:
* public class object final void robotclassobjectfinal{
*      controller.main.public.alpha.beta.epsilon.epislon.axes.axises.axis1.speed.speedValue =loudTyping.lound.loud.thing.class.private.public.static.dynamic.void.int.double.float.variableType.one.two.three.finalXYZ;
*
*}
*/


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // ++ JOYSTICK STUFF ========================================
  // ++ we make a JoyUtil object instead of an XboxController object; JoyUtil inherits XboxController
  public static JoyUtil primaryController = new JoyUtil( Constants.Joysticks.primaryControllerID );
  public static JoyUtil secondaryController = new JoyUtil( Constants.Joysticks.secondaryControllerID );
    public static JoystickButton secondaryRightBumper = new JoystickButton(secondaryController, Constants.Joysticks.RightBumper);
    public static JoystickButton secondaryXButton = new JoystickButton(secondaryController, Constants.Joysticks.X);
    public static JoystickButton primaryAButton = new JoystickButton(primaryController, Constants.Joysticks.A);
    public static JoystickButton autoShootRoutineButton = new JoystickButton(secondaryController, Constants.Joysticks.Y);



  // ++ SUBSYSTEMS AND COMMANDS ========================================
  // SUBSYSTEMS -------------------
    // ++ "utility subsystems"
  private final IMUSubsystem m_IMUSubsystem = new IMUSubsystem();
  private final ShuffleboardSubsystem m_Shuffleboardsubsystem = new ShuffleboardSubsystem();

    // ++ robot subsystems
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final IntakeIndexerSubsystem m_IntakeIndexerSubsystem = new IntakeIndexerSubsystem();
  // COMMANDS--------------------
    // ++ teleop commands
  private final TeleopPIDDriveCommand m_PIDDriveCommand = new TeleopPIDDriveCommand(m_DriveSubsystem, primaryController);
  private final ShooterCommand m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem, secondaryController);
  private final AcceptCommand m_AcceptCommand = new AcceptCommand(m_IntakeIndexerSubsystem, null);
  private final RebuffCommand m_RebuffCommand = new RebuffCommand(m_IntakeIndexerSubsystem);
  private final SpinTakeCommand m_SpinIntakeCommand = new SpinTakeCommand(m_IntakeIndexerSubsystem, secondaryController);

  private final ShootRoutineCommandGroup m_AutoShootCommand = new ShootRoutineCommandGroup(
    m_DriveSubsystem,
    m_LimelightSubsystem,
    m_IntakeIndexerSubsystem,
    m_ShooterSubsystem
    );

    // ++ auto commands
  private final AutonomousPIDTaxiCommand m_AutonomousPIDTaxiCommand = new AutonomousPIDTaxiCommand(m_DriveSubsystem);

    // ++ shooter routine commands
  private final LimelightAlignDriveCommand m_LimeDriveCommand = new LimelightAlignDriveCommand(m_DriveSubsystem, m_LimelightSubsystem);

  // ++ END SUBSYSTEMS/COMMANDS ===============================================


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // ++ command stuff
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
    primaryAButton.whenPressed(m_LimeDriveCommand);
    autoShootRoutineButton.whenPressed(m_AutoShootCommand);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutonomousPIDTaxiCommand(m_DriveSubsystem);
    // return null;
  }
}
