// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopPIDDriveCommand extends CommandBase {

  private final DriveSubsystem m_DriveSubsystem;
  private final XboxController controller;
  private final MecanumDriveKinematics kinematics;


  /** Creates a new TeleopPIDCommand. */
  public TeleopPIDDriveCommand(DriveSubsystem subsystem, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveSubsystem = subsystem;
    this.controller = controller;

    kinematics = new MecanumDriveKinematics(Constants.DriveTrain.frontLeftMeters, Constants.DriveTrain.frontRightMeters, Constants.DriveTrain.backLeftMeters, Constants.DriveTrain.backRightMeters);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds vehicleSpeed = new ChassisSpeeds(controller.getRawAxis(Constants.Joysticks.LeftJoystickX), -controller.getRawAxis(Constants.Joysticks.LeftJoystickY), controller.getRawAxis(Constants.Joysticks.RightJoystickX));
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(vehicleSpeed);
    wheelSpeeds.desaturate(1);
    m_DriveSubsystem.setVelocityReference(wheelSpeeds.frontLeftMetersPerSecond, wheelSpeeds.frontRightMetersPerSecond, wheelSpeeds.rearLeftMetersPerSecond, wheelSpeeds.rearRightMetersPerSecond);

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
