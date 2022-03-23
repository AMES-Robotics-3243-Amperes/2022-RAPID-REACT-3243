// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//  ++ FRC stuff
package frc.robot;


// ++ project stuff
import frc.robot.Constants; 
import edu.wpi.first.wpilibj2.command.Command;

// ++ misc
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.IMUSubsystem;

// ++ network tables / shuffleboard stuff
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// ++ SUBSYSTEMS
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeIndexerSubsystem;


// ++ COMMANDS
  // ++ teleop
import frc.robot.commands.TeleopPIDDriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.RebuffCommand;
import frc.robot.commands.AcceptCommand;
import frc.robot.commands.SpinIntakeCommand;

  // ++ auto
import frc.robot.commands.AutonomousPIDTaxiCommand;


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



  // ++ SUBSYSTEMS AND COMMANDS ========================================
  // SUBSYSTEMS -------------------
    // ++ robot subsystems
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final IntakeIndexerSubsystem m_IntakeIndexerSubsystem = new IntakeIndexerSubsystem();
    // ++ "utility subsystems"
  private final IMUSubsystem m_IMUSubsystem = new IMUSubsystem();
  private final ShuffleboardSubsystem m_Shuffleboardsubsystem = new ShuffleboardSubsystem();
  // COMMANDS--------------------
    // ++ teleop commands
  private final ShooterCommand m_ShooterCommand = new ShooterCommand(m_ShooterSubsystem, secondaryController);
  private final AcceptCommand m_AcceptCommand = new AcceptCommand(m_IntakeIndexerSubsystem, Constants.IntakeIndexer.acceptRotations);
  private final RebuffCommand m_RebuffCommand = new RebuffCommand(m_IntakeIndexerSubsystem, Constants.IntakeIndexer.rebuffRotations, Constants.IntakeIndexer.rebuffSpeed, Constants.IntakeIndexer.rebuffDuration);
  private final SpinIntakeCommand m_SpinIntakeCommand = new SpinIntakeCommand(m_IntakeIndexerSubsystem, primaryController);
    // ++ auto commands
  private final TeleopPIDDriveCommand m_PIDDriveCommand = new TeleopPIDDriveCommand(m_DriveSubsystem, primaryController);
  // private final AutonomousPIDTaxiCommand m_AutonomousPIDTaxiCommand = new AutonomousPIDTaxiCommand(m_DriveSubsystem);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // ++ command stuff
    m_DriveSubsystem.setDefaultCommand(m_PIDDriveCommand);
    m_ShooterSubsystem.setDefaultCommand(m_ShooterCommand);



  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
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
