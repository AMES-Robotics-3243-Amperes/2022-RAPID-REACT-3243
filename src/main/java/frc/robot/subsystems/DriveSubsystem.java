// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {

  // ~~++ create motor objects
  private final CANSparkMax frontLeftMotor = new CANSparkMax( Constants.DriveTrain.frontLeftID, MotorType.kBrushless);
  private final CANSparkMax frontRightMotor = new CANSparkMax( Constants.DriveTrain.frontRightID, MotorType.kBrushless);
  private final CANSparkMax backLeftMotor = new CANSparkMax( Constants.DriveTrain.backLeftID, MotorType.kBrushless);
  private final CANSparkMax backRightMotor = new CANSparkMax( Constants.DriveTrain.backRightID, MotorType.kBrushless);

  // ~~ create encoder objects
  private final RelativeEncoder frontLeftEncoder;
  private final RelativeEncoder frontRightEncoder;
  private final RelativeEncoder backLeftEncoder;
  private final RelativeEncoder backRightEncoder;

  // ~~ create PID controller objects
  private final SparkMaxPIDController frontLeftPIDController;
  private final SparkMaxPIDController frontRightPIDController;
  private final SparkMaxPIDController backLeftPIDController;
  private final SparkMaxPIDController backRightPIDController;

  // ~~ Field Object for visulaization in shuffleboard or simulation
  // Field2d field = new Field2d();

  // ~~ Pose object for keeping track of robot position
  // Pose2d pose = new Pose2d(6.0, 4.0, new Rotation2d());

  // ~~ mecanum drive kinematics object for calculating wheel speeds and positions from chassis speeds and positions
  MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    Constants.DriveTrain.frontLeftMeters, 
    Constants.DriveTrain.frontRightMeters, 
    Constants.DriveTrain.backLeftMeters, 
    Constants.DriveTrain.backRightMeters
  );

  // ~~ mecanum drive odometry object for calculating position of robot based on wheel speeds
  // MecanumDriveOdometry odometry;

  // ~~ Shuffleboard
  private final ShuffleboardTab pidTab = Shuffleboard.getTab("PID Tuning");
  public NetworkTableEntry pGain;
  public NetworkTableEntry iGain;
  public NetworkTableEntry dGain;
  public NetworkTableEntry speedErrorThreshold;

  private SimpleWidget pGainWidget;
  private SimpleWidget iGainWidget;
  private SimpleWidget dGainWidget;
  private SimpleWidget speedErrorThresholdWidget;


  private NetworkTableEntry tGyroYaw;
  private NetworkTableEntry tXPos;
  private NetworkTableEntry tYPos;
  private NetworkTableEntry tXSpeed;
  private NetworkTableEntry tYSpeed;

  // private double gyroOffset = 0.0;
  // private double xChange = 0.0;
  // private double yChange = 0.0;


  public DriveSubsystem() {
    // ++ invert the motors that need to be inverted (both sides are here so robot doesn't need to be 
    // ++ restarted every time the settings are adjusted)
    // ++ LEFT  \/
    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);
    // ++ RIGHT \/
    frontRightMotor.setInverted(false);
    backRightMotor.setInverted(false);
    // ++ -------------------


    
    // MecanumDrive drivetrain = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();

    // frontLeftEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
    // frontRightEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
    // backLeftEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
    // backRightEncoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);

    // frontLeftEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
    // frontRightEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
    // backLeftEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
    // backRightEncoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);

    frontLeftPIDController = frontLeftMotor.getPIDController();
    frontRightPIDController = frontRightMotor.getPIDController();
    backLeftPIDController = backLeftMotor.getPIDController();
    backRightPIDController = backRightMotor.getPIDController();

    pGainWidget = pidTab.add("P gain", Constants.DriveTrain.teleopPGain);
    iGainWidget = pidTab.add("I gain", Constants.DriveTrain.teleopIGain);
    dGainWidget = pidTab.add("D gain", Constants.DriveTrain.teleopDGain);
    speedErrorThresholdWidget = pidTab.add("Speed Error Tolerance", 0.1);

    // tGyroYaw = Shuffleboard.getTab("IMU").add("yaw", 0.0).getEntry();
    // tXPos = Shuffleboard.getTab("IMU").add("x position", 0.0).getEntry();
    // tYPos = Shuffleboard.getTab("IMU").add("y position", 0.0).getEntry();
    // tXSpeed = Shuffleboard.getTab("IMU").add("x velocity", 0.0).getEntry();
    // tYSpeed = Shuffleboard.getTab("IMU").add("y velocity", 0.0).getEntry();

    // resetGyroRotation();

    // resetPose();

    // odometry = new MecanumDriveOdometry(kinematics, getGyroRotation(), pose);

  }


  // ++ IMPORTANT ONE THAT ACTUALLY DOES THE DRIVING
  // ~~ Sets the velocity reference of the 4 PID loops, for driving in teleop
  public void setVelocityReference (double flRef, double frRef, double blRef, double brRef) {
    
    // frontLeftMotor.set(flRef);
    // frontRightMotor.set(frRef);
    // backLeftMotor.set(blRef);
    // backRightMotor.set(brRef);

    frontLeftPIDController.setReference(flRef, ControlType.kVelocity);
    frontRightPIDController.setReference(frRef, ControlType.kVelocity);
    backLeftPIDController.setReference(blRef, ControlType.kVelocity);
    backRightPIDController.setReference(brRef, ControlType.kVelocity);

    SmartDashboard.putNumber("FL target speed", flRef);
    SmartDashboard.putNumber("FR target speed", frRef);
    SmartDashboard.putNumber("BL target speed", blRef);
    SmartDashboard.putNumber("BR target speed", brRef);


    SmartDashboard.putNumber("FL ACTUAL speed", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("FR ACTUAL speed", frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("BL ACTUAL speed", backLeftEncoder.getVelocity());
    SmartDashboard.putNumber("BR ACTUAL speed", backRightEncoder.getVelocity());
  
    // speeds.feed();
  }

  // ~~ Sets the P, I, and D gains of the 4 PID loops
  public void setPIDValues(double kP, double kI, double kD) {
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


  public void getShuffleboardPID() {
    pGain = pGainWidget.getEntry();
    iGain = iGainWidget.getEntry();
    dGain = dGainWidget.getEntry();
  }






  // // ~~ returns a Rotation2d object with the robot's current angle, in radians
  // public Rotation2d getGyroRotation() {
  //   double angle = tGyroYaw.getDouble(0.0);
  //   angle *= (Math.PI / 180);
  //   angle -= gyroOffset;
  //   Rotation2d rotation = new Rotation2d(angle);
  //   return rotation;
  // }

  // public void resetGyroRotation() {
  //   gyroOffset = getGyroRotation().getRadians();
  // }

  // ~~ resets the Pose2d and encoder positions of all the motors
  // public void resetPose() {
  //   pose = new Pose2d(6.0, 4.0, new Rotation2d());
  //   ChassisSpeeds chassisPos = new ChassisSpeeds(pose.getX(), pose.getY(), pose.getRotation().getRadians());
  //   MecanumDriveWheelSpeeds wheelPos = kinematics.toWheelSpeeds(chassisPos);
  //   frontLeftEncoder.setPosition(wheelPos.frontLeftMetersPerSecond);
  //   frontRightEncoder.setPosition(wheelPos.frontRightMetersPerSecond);
  //   backLeftEncoder.setPosition(wheelPos.rearLeftMetersPerSecond);
  //   backRightEncoder.setPosition(wheelPos.rearRightMetersPerSecond);

    // IMPORTANT - we may need to set motor positional PID loop to the encoder position after resetting to prevent runaway robots.
    // Implementation would be:
    // frontLeftPIDController.setReference(frontLeftEncoder.getPosition(), ControlType.kPosition);
    // frontRightPIDController.setReference(frontRightEncoder.getPosition(), ControlType.kPosition);
    // backLeftPIDController.setReference(backLeftEncoder.getPosition(), ControlType.kPosition);
    // backRightPIDController.setReference(backRightEncoder.getPosition(), ControlType.kPosition);

  // }

  // // ~~ changes the robots position based off of current position
  // public void changeRobotPosition(Pose2d transform) {
  //   ChassisSpeeds chassisTransform = new ChassisSpeeds(transform.getX(), transform.getY(), transform.getRotation().getRadians());
  //   MecanumDriveWheelSpeeds wheelTransform = kinematics.toWheelSpeeds(chassisTransform);

  //   double frontLeftPos = frontLeftEncoder.getPosition() + wheelTransform.frontLeftMetersPerSecond;
  //   double frontRightPos = frontRightEncoder.getPosition() + wheelTransform.frontRightMetersPerSecond;
  //   double backLeftPos = backLeftEncoder.getPosition() + wheelTransform.rearLeftMetersPerSecond;
  //   double backRightPos = backRightEncoder.getPosition() + wheelTransform.rearRightMetersPerSecond;

  //   frontLeftPIDController.setReference(frontLeftPos, ControlType.kPosition);
  //   frontRightPIDController.setReference(frontRightPos, ControlType.kPosition);
  //   backLeftPIDController.setReference(backLeftPos, ControlType.kPosition);
  //   backRightPIDController.setReference(backRightPos, ControlType.kPosition);
  // }

  // // ~~ sets the absolute robot position
  // public void setRobotPosition(Pose2d position) {
  //   ChassisSpeeds chassisPos = new ChassisSpeeds(position.getX(), position.getY(), position.getRotation().getRadians());
  //   MecanumDriveWheelSpeeds wheelPos = kinematics.toWheelSpeeds(chassisPos);

  //   double frontLeftPos = wheelPos.frontLeftMetersPerSecond;
  //   double frontRightPos = wheelPos.frontRightMetersPerSecond;
  //   double backLeftPos = wheelPos.rearLeftMetersPerSecond;
  //   double backRightPos = wheelPos.rearRightMetersPerSecond;

  //   frontLeftPIDController.setReference(frontLeftPos, ControlType.kPosition);
  //   frontRightPIDController.setReference(frontRightPos, ControlType.kPosition);
  //   backLeftPIDController.setReference(backLeftPos, ControlType.kPosition);
  //   backRightPIDController.setReference(backRightPos, ControlType.kPosition);
  // }

  // ~~ Updates the PID tuning widgets on shuffleboard





  // public void driveCartesian (double X_speed, double Y_speed, double Z_rotation) {
  //   //++ I think this method is now redundant with PID stuff?
  //   speeds.driveCartesian(-Y_speed, X_speed, Z_rotation);
  // }


  @Override
  public void periodic() {

    // xChange = tXPos.getDouble(0.0) - xChange;
    // yChange = tYPos.getDouble(0.0) - yChange;

    // MecanumDriveWheelSpeeds wheelspeeds = new MecanumDriveWheelSpeeds(
    //     frontLeftEncoder.getVelocity(),
    //     frontRightEncoder.getVelocity(),
    //     backLeftEncoder.getVelocity(),
    //     backRightEncoder.getVelocity()
    // );
    // speedErrorThreshold = speedErrorThresholdWidget.getEntry();
    // // ~~ Use odometry object for calculating position
    // ChassisSpeeds expectedSpeed = kinematics.toChassisSpeeds(wheelspeeds);
    // ChassisSpeeds actualSpeed = new ChassisSpeeds(tXSpeed.getDouble(0.0), tYSpeed.getDouble(0.0), 0.0);

    // Double xError = expectedSpeed.vxMetersPerSecond - actualSpeed.vxMetersPerSecond;
    // Double yError = expectedSpeed.vyMetersPerSecond - actualSpeed.vyMetersPerSecond;
    
    // Double speedError = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

    // SmartDashboard.putNumber("Speed Error", speedError);

    // ~~ Checks if the robot's wheels are slipping to determine if odometry or the imu would be more accurate
    // if (speedError < speedErrorThreshold.getDouble(0.1)) {
    //   // ~~ Calculates position based on odometry
    //   pose = odometry.update(getGyroRotation(), wheelspeeds);

    //   SmartDashboard.putBoolean("Slipping?", false);
    // } 
    // else {
    //   // ~~ Calculates position based on imu
    //   Double newX = pose.getX() + xChange;
    //   Double newY = pose.getY() + yChange;
    //   Rotation2d newR = pose.getRotation();

    //   pose = new Pose2d(newX, newY, newR);

    //   // ~~ Updates odometry object with data from imu
    //   odometry.update(getGyroRotation(), wheelspeeds);
    //   odometry.resetPosition(pose, getGyroRotation());

    //   SmartDashboard.putBoolean("Slipping?", true);

    // }

  //   SmartDashboard.putNumber("Robot x", pose.getX());
  //   SmartDashboard.putNumber("Robot y", pose.getY());
  //   SmartDashboard.putNumber("Robot rotation", pose.getRotation().getDegrees());

  //   SmartDashboard.putNumber("FL Speed", frontLeftEncoder.getVelocity());
  //   SmartDashboard.putNumber("FL Position", frontLeftEncoder.getPosition());
  //   // ~~ Update field object for shuffleboard
  //   field.setRobotPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}