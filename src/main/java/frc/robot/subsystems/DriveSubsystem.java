// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // ++ Gyro and Acceleromter
  

  // ++ Field Object for visulaization in shuffleboard or simulation
  Field2d field = new Field2d();

  // ++ mecanum drive kinematics object
  MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    Constants.DriveTrain.frontLeftMeters, Constants.DriveTrain.frontRightMeters, Constants.DriveTrain.backLeftMeters, Constants.DriveTrain.backRightMeters
  );


  // ++ Shuffleboard
  private final ShuffleboardTab pidTab = Shuffleboard.getTab("PID Tuning");
  public NetworkTableEntry pGain;
  public NetworkTableEntry iGain;
  public NetworkTableEntry dGain;

  private SimpleWidget pGainWidget;
  private SimpleWidget iGainWidget;
  private SimpleWidget dGainWidget;

  public DriveSubsystem() {
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
    // speeds = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();

    frontLeftEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
    frontRightEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
    backLeftEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
    backRightEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);

    frontLeftEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
    frontRightEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
    backLeftEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
    backRightEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);

    frontLeftPIDController = frontLeftMotor.getPIDController();
    frontRightPIDController = frontRightMotor.getPIDController();
    backLeftPIDController = backLeftMotor.getPIDController();
    backRightPIDController = backRightMotor.getPIDController();

    pGainWidget = pidTab.add("P gain", 1.0);
    iGainWidget = pidTab.add("I gain", 0.0);
    dGainWidget = pidTab.add("D gain", 0.0);

  }

  public void driveCartesian (double X_speed, double Y_speed, double Z_rotation) {
    // speeds.driveCartesian(-Y_speed, X_speed, Z_rotation);
  }

  public void getShuffleboardPID() {
    pGain = pGainWidget.getEntry();
    iGain = iGainWidget.getEntry();
    dGain = dGainWidget.getEntry();
  }

  public void setVelocityReference (double flRef, double frRef, double blRef, double brRef) {
    
    // frontLeftMotor.set(flRef);
    // frontRightMotor.set(frRef);
    // backLeftMotor.set(blRef);
    // backRightMotor.set(brRef);

    frontLeftPIDController.setReference(flRef, ControlType.kVelocity);
    frontRightPIDController.setReference(frRef, ControlType.kVelocity);
    backLeftPIDController.setReference(blRef, ControlType.kVelocity);
    backRightPIDController.setReference(brRef, ControlType.kVelocity);

    SmartDashboard.putNumber("FL Speed", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("FL Position", frontLeftEncoder.getPosition());

    // speeds.feed();
  }

  public void setPID(double kP, double kI, double kD) {
    frontLeftPIDController.setP(kP);
    frontLeftPIDController.setI(kI);
    frontLeftPIDController.setD(kD);
    
    frontRightPIDController.setP(kP);
    frontRightPIDController.setI(kI);
    frontRightPIDController.setD(kD);
    
    backLeftPIDController.setP(kP);
    backLeftPIDController.setI(kI);
    backLeftPIDController.setD(kD);
    
    backRightPIDController.setP(kP);
    backRightPIDController.setI(kI);
    backRightPIDController.setD(kD);

    frontLeftPIDController.setIAccum(0);
    frontRightPIDController.setIAccum(0);
    backLeftPIDController.setIAccum(0);
    backRightPIDController.setIAccum(0);
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
