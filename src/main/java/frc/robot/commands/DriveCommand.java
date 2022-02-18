// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.JoyUtil;
import frc.robot.Constants; 

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_DriveSubsystem;
  private final JoyUtil joystick;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem, JoyUtil driveController) {
    m_DriveSubsystem = subsystem;
    joystick = driveController; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("DriveCommand in initialize");
    joystick.zeroPreviousFiltered();

    // ++ these are here in initialize() so they're set to 0 every time the method is scheduled
    // ++ that's potentially important because this command might stop being scheduled for some reason, so when it starts again,
    // ++ the previous value should be 0

  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("DriveComand in execute");

    SmartDashboard.putNumber("prevFilteredJoyX", prevFilteredJoyX);
    SmartDashboard.putNumber("prevFilteredJoyY", prevFilteredJoyY);
    // ++ we want to curve the x and y speeds the same, but we'll probably do the rotation differently
    double speedX =  JoyUtil.composeDriveJoyFunctions( joystick.getLeftX(), prevFilteredJoyX );
    double speedY =  JoyUtil.composeDriveJoyFunctions( joystick.getLeftY(), prevFilteredJoyY );

    // ++ these set the old "previous" values to the most recent previous values, which are then used the next time this runs
    prevFilteredJoyX = JoyUtil.driveLowPassFilter( JoyUtil.posWithDeadzone(joystick.getLeftX()), prevFilteredJoyX );
    prevFilteredJoyY = JoyUtil.driveLowPassFilter( JoyUtil.posWithDeadzone(joystick.getLeftY()), prevFilteredJoyY );



    // ++ all these methods are defined in the JoyUtil class
    // ++ DRIVE 
    double speedX = joystick.getDriveXWithAdjustments();
    double speedY = joystick.getDriveYWithAdjustments(); 
    // ++ ROTATION
    double speedR = joystick.getRotationWithAdjustments();
    // ++ SET -- input speeds in the order X, Y, and R
    m_DriveSubsystem.setMotors(speedX, speedY, speedR); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
