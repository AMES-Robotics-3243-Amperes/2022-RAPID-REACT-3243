// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//  ++ FRC stuff
package frc.robot;


// ++ project stuff
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// ++ misc

// ++ SUBSYSTEMS
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShuffleboardSubsystem;


// ++ COMMANDS
  // ++ teleop
import frc.robot.commands.TeleopPIDDriveCommand;
// ++ auto
// import frc.robot.commands.AutonomousPIDTaxiCommand;



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



  // ++ SUBSYSTEMS AND COMMANDS ========================================
  // SUBSYSTEMS -------------------
    // ++ robot subsystems
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
    // ++ "utility subsystems"
  private final ShuffleboardSubsystem m_Shuffleboardsubsystem = new ShuffleboardSubsystem();
  
  // COMMANDS--------------------
    // ++ teleop commands
  private final TeleopPIDDriveCommand m_PIDDriveCommand = new TeleopPIDDriveCommand(m_DriveSubsystem, primaryController);

    // ++ auto commands
  // private final AutonomousPIDTaxiCommand m_AutonomousPIDTaxiCommand = new AutonomousPIDTaxiCommand(m_DriveSubsystem);
  // ++ END SUBSYSTEMS/COMMANDS ===============================================


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // ++ command stuff
    m_DriveSubsystem.setDefaultCommand(m_PIDDriveCommand);

    configureButtonBindings();

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
    // return new AutonomousPIDTaxiCommand(m_DriveSubsystem);
    return null;
  }
}
