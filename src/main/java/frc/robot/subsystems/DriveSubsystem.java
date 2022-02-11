// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {

  // ++ create motor objects
  private final CANSparkMax frontLeftMotor = new CANSparkMax( Constants.DriveTrain.frontLeftID, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax( Constants.DriveTrain.frontRightID, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax( Constants.DriveTrain.backLeftID, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax( Constants.DriveTrain.backRightID, MotorType.kBrushless);

  // ++ create encoder objects
  private final RelativeEncoder frontLeftEncoder;
  private final RelativeEncoder frontRightEncoder;
  private final RelativeEncoder backLeftEncoder;
  private final RelativeEncoder backRightEncoder;

  // ++ create PID controller objects
  private final SparkMaxPIDController frontLeftPIDController;
  private final SparkMaxPIDController frontRightPIDController;
  private final SparkMaxPIDController backLeftPIDController;
  private final SparkMaxPIDController backRightPIDController;

  // ++ create mecanum drive object
  private MecanumDrive speeds;

  
  public DriveSubsystem() {
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
    speeds = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();

    frontLeftPIDController = frontLeftMotor.getPIDController();
    frontRightPIDController = frontRightMotor.getPIDController();
    backLeftPIDController = backLeftMotor.getPIDController();
    backRightPIDController = backRightMotor.getPIDController();

    frontLeftPIDController.setP(Constants.DriveTrain.kP);
    frontLeftPIDController.setI(Constants.DriveTrain.kI);
    frontLeftPIDController.setD(Constants.DriveTrain.kD);
    
    frontRightPIDController.setP(Constants.DriveTrain.kP);
    frontRightPIDController.setI(Constants.DriveTrain.kI);
    frontRightPIDController.setD(Constants.DriveTrain.kD);
    
    backLeftPIDController.setP(Constants.DriveTrain.kP);
    backLeftPIDController.setI(Constants.DriveTrain.kI);
    backLeftPIDController.setD(Constants.DriveTrain.kD);
    
    backRightPIDController.setP(Constants.DriveTrain.kP);
    backRightPIDController.setI(Constants.DriveTrain.kI);
    backRightPIDController.setD(Constants.DriveTrain.kD);

  }

  public void driveCartesian (double X_speed, double Y_speed, double Z_rotation) {
    speeds.driveCartesian(Y_speed, X_speed, Z_rotation);
  }

  public void setVelocityReference (double flRef, double frRef, double blRef, double brRef) {
    frontLeftPIDController.setReference(flRef, ControlType.kVelocity);
    frontRightPIDController.setReference(frRef, ControlType.kVelocity);
    backLeftPIDController.setReference(blRef, ControlType.kVelocity);
    backRightPIDController.setReference(brRef, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
