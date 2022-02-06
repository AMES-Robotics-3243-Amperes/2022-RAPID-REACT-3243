// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MecanumDriveSubsystem extends SubsystemBase {
  /** Creates a new MechanumDriveSubsystem. */
  private MecanumDrive MecDriveMain;


  public MecanumDriveSubsystem() {
    
    //*Motor definition here:
    final CANSparkMax RightFrontMotor = new CANSparkMax(Constants.MecanumDrive.FrontRightMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
    final CANSparkMax RightBackMotor = new CANSparkMax(Constants.MecanumDrive.BackRightMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
    final CANSparkMax LeftFrontMotor = new CANSparkMax(Constants.MecanumDrive.FrontLeftMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
    final CANSparkMax LeftBackMotor = new CANSparkMax(Constants.MecanumDrive.BackLeftMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
    //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

    //*Mechanum Drive here:
    MecDriveMain = new MecanumDrive(LeftFrontMotor, LeftBackMotor, RightFrontMotor, RightBackMotor);
    //*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

  }

  public void MecDrive(double RstickY, double RstickX, double LstickX) {
    MecDriveMain.driveCartesian(RstickY, RstickX, LstickX);
    }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
