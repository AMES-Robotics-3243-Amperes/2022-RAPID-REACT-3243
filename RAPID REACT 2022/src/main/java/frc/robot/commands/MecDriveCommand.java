// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MecanumDriveSubsystem;

public class MecDriveCommand extends CommandBase {
  /** Creates a new MecDriveCommand. */
  //Variable Declaration Here: 
  private PS4Controller JoystickController0;
  private MecanumDriveSubsystem mDrive;
  private double RightY;
  private double RightX;
  private double LeftX;

  public MecDriveCommand(PS4Controller Joystick0, MecanumDriveSubsystem MainDrive) {
    JoystickController0 = Joystick0;
    mDrive = MainDrive;
    addRequirements(MainDrive); 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RightY = JoystickController0.getRightY();
    RightX = JoystickController0.getRightX();
    LeftX = JoystickController0.getLeftX();
    mDrive.MecDrive(RightY, RightX, LeftX);
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
