// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  static Field2d field = new Field2d();
  // // ++ IMU subsystem object
  // IMUSubsystem IMUSubsystem = new IMUSubsystem();

  // ~~ Pose object for keeping track of robot position
  Pose2d pose = new Pose2d(6.0, 4.0, new Rotation2d());
  double frontLeftTarget;
  double frontRightTarget;
  double backLeftTarget;
  double backRightTarget;

  // ~~ mecanum drive kinematics object for calculating wheel speeds and positions from chassis speeds and positions
  MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    Constants.DriveTrain.frontLeftMeters, 
    Constants.DriveTrain.frontRightMeters, 
    Constants.DriveTrain.backLeftMeters, 
    Constants.DriveTrain.backRightMeters
  );

  // ~~ mecanum drive odometry object for calculating position of robot based on wheel speeds
  MecanumDriveOdometry odometry;

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

  private double gyroOffset = 0.0;
  private double xChange = 0.0;
  private double yChange = 0.0;


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

    pGainWidget = pidTab.add("P gain", Constants.DriveTrain.teleopPGain);
    iGainWidget = pidTab.add("I gain", Constants.DriveTrain.teleopIGain);
    dGainWidget = pidTab.add("D gain", Constants.DriveTrain.teleopDGain);
    speedErrorThresholdWidget = pidTab.add("Speed Error Tolerance", 0.1);


    // ~~ Positional Stuff ============================================================


    resetPose(0.0, 0.0, 0.0);

    odometry = new MecanumDriveOdometry(kinematics, IMUSubsystem.getGyroRotation(), pose);

    frontLeftTarget = 0.0;
    frontRightTarget = 0.0;
    backLeftTarget = 0.0;
    backRightTarget = 0.0;

    // ~~ =============================================================================

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

    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("FL target speed", flRef);
      SmartDashboard.putNumber("FR target speed", frRef);
      SmartDashboard.putNumber("BL target speed", blRef);
      SmartDashboard.putNumber("BR target speed", brRef);

      //annette be cool
      SmartDashboard.putNumber("FL ACTUAL speed", frontLeftEncoder.getVelocity());
      SmartDashboard.putNumber("FR ACTUAL speed", frontRightEncoder.getVelocity());
      SmartDashboard.putNumber("BL ACTUAL speed", backLeftEncoder.getVelocity());
      SmartDashboard.putNumber("BR ACTUAL speed", backRightEncoder.getVelocity());
    }
  
    // speeds.feed();
  }

  public void setVelocityReference(MecanumDriveWheelSpeeds wheelSpeeds) {
    setVelocityReference(
      wheelSpeeds.frontLeftMetersPerSecond,
      wheelSpeeds.frontRightMetersPerSecond,
      wheelSpeeds.rearLeftMetersPerSecond,
      wheelSpeeds.rearRightMetersPerSecond
    );
  }

  public void setPositionalReference(double flRef, double frRef, double blRef, double brRef) {
    frontLeftPIDController.setReference(flRef, ControlType.kPosition);
    frontRightPIDController.setReference(frRef, ControlType.kPosition);
    backLeftPIDController.setReference(blRef, ControlType.kPosition);
    backRightPIDController.setReference(brRef, ControlType.kPosition);
    frontLeftTarget = flRef;
    frontRightTarget = frRef;
    backLeftTarget = blRef;
    backRightTarget = brRef;
  }

  public void setPositionalReference(MecanumDriveWheelSpeeds wheelPositions) {
    setPositionalReference(
      wheelPositions.frontLeftMetersPerSecond,
      wheelPositions.frontRightMetersPerSecond,
      wheelPositions.rearLeftMetersPerSecond,
      wheelPositions.rearRightMetersPerSecond
    );
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

  // ++ maybe move this to ShuffleboardSubsystem?
  public void getShuffleboardPID() {
    pGain = pGainWidget.getEntry();
    iGain = iGainWidget.getEntry();
    dGain = dGainWidget.getEntry();
  }


  // ++ THE THREE METHODS BELOW ARE USED TO GET THE SPEEDS OF EACH WHEEL FOR THE MECHANUM DRIVE

  // ~~ gets the kinematics object for inverse kinematics in commands
  public MecanumDriveWheelSpeeds toWheelSpeeds(ChassisSpeeds robotSpeed) {
    return kinematics.toWheelSpeeds(robotSpeed);
  }
  
  public ChassisSpeeds getTeleopChassisSpeed(double x, double y, double r) {
    // ++ outputs a vehicleSpeed object based on X, Y, and rotation values
    ChassisSpeeds vehicleSpeed = new ChassisSpeeds(y, x, r);
      return vehicleSpeed;
  }


    /**
   * ++ this takes x, y, and r values (maybe from controller) 
   * and creates a wheelSpeeds object, which can then be used to set PIDs etc.
   * 
   * (this explination thing might also be broken; it's just a test to figure out how it works)
   *
   * @param x x velocity
   * @param y y velocity
   * @param r rotation
   * @return a wheelSpeeds object
   */
  public MecanumDriveWheelSpeeds getWheelSpeeds(double x, double y, double r) {
    ChassisSpeeds vehicleSpeed = getTeleopChassisSpeed(x, y, r);
    MecanumDriveWheelSpeeds wheelSpeeds = toWheelSpeeds(vehicleSpeed);
    wheelSpeeds.desaturate(Constants.DriveTrain.maxWheelSpeed);
    return wheelSpeeds;
  }









  // ~~ resets the Pose2d and encoder positions of all the motors
  public void resetPose(Pose2d newPose) {
    IMUSubsystem.setYaw(newPose.getRotation().getDegrees());
    pose = newPose;
    setPositionalReference(0.0, 0.0, 0.0, 0.0);
    frontLeftEncoder.setPosition(0.0);
    frontRightEncoder.setPosition(0.0);
    backLeftEncoder.setPosition(0.0);
    backRightEncoder.setPosition(0.0);

    // IMPORTANT - we may need to set motor positional PID loop to the encoder position after resetting to prevent runaway robots.
    // Implementation would be:
    // frontLeftPIDController.setReference(frontLeftEncoder.getPosition(), ControlType.kPosition);
    // frontRightPIDController.setReference(frontRightEncoder.getPosition(), ControlType.kPosition);
    // backLeftPIDController.setReference(backLeftEncoder.getPosition(), ControlType.kPosition);
    // backRightPIDController.setReference(backRightEncoder.getPosition(), ControlType.kPosition);

  }

  public void resetPose(double x, double y, double r) {
    resetPose(new Pose2d(x, y, new Rotation2d()));
  }


  // ~~ changes the robots position based off of current position
  public void changeRobotPosition(Pose2d transform) {
    ChassisSpeeds chassisPos = new ChassisSpeeds(transform.getX(), transform.getY(), 0);
    MecanumDriveWheelSpeeds wheelPos = kinematics.toWheelSpeeds(chassisPos);

    frontLeftEncoder.setPosition(0.0);
    frontRightEncoder.setPosition(0.0);
    backLeftEncoder.setPosition(0.0);
    backRightEncoder.setPosition(0.0);

    setPositionalReference(wheelPos);
  }

  // // ~~ sets the absolute robot position
  public void setRobotPosition(Pose2d position) {
    Pose2d transform = new Pose2d(position.getX() - pose.getX(), position.getY() - pose.getY(), new Rotation2d(0.0));
    changeRobotPosition(transform);
  }

  public void changeRobotAngle(double radians) {
    ChassisSpeeds chassisPos = new ChassisSpeeds(0,0, radians);
    MecanumDriveWheelSpeeds wheelPos = kinematics.toWheelSpeeds(chassisPos);

    frontLeftEncoder.setPosition(0.0);
    frontRightEncoder.setPosition(0.0);
    backLeftEncoder.setPosition(0.0);
    backRightEncoder.setPosition(0.0);

    setPositionalReference(wheelPos);
  }

  // ~~ Sets the angle of the robot in radians from -π to π
  public void setRobotAngle(double radians) {
    // ~~ Dumb math to make the angle of the robot continuous instead of linear 
    // ~~ (ie. being at π and rotating π radians puts you at 0, not 2π)
    // ~~ Also means the robot wont do a 360 just to turn from slightly left to slightly right
    double currentAngle = IMUSubsystem.getGyroRotation().getRadians();
    double standardDif = Math.abs(radians - currentAngle);
    double highDif = Math.abs((radians + (2 * Math.PI)) - currentAngle);
    double lowDif = Math.abs((radians - (2 * Math.PI)) - currentAngle);
    double lowestDif = Math.min(standardDif, Math.min(lowDif, highDif));

    if (lowestDif == standardDif) {
      changeRobotAngle(radians - currentAngle);
    } else if (lowestDif == lowDif) {
      changeRobotAngle((radians + (2 * Math.PI)) - currentAngle);
    }else {
      changeRobotAngle((radians + (2 * Math.PI)) - currentAngle);
    }
  }

  public void lookAt(Pose2d target) {
    Pose2d vector = target.relativeTo(pose);
    double radians = Math.atan2(vector.getY(), vector.getX());
    setRobotAngle(radians);
  }

  public boolean atTargetPosition() {
    double flError = Math.abs(frontLeftTarget - frontLeftEncoder.getPosition());
    double frError = Math.abs(frontRightTarget - frontRightEncoder.getPosition());
    double blError = Math.abs(backLeftTarget - backLeftEncoder.getPosition());
    double brError = Math.abs(backRightTarget - backRightEncoder.getPosition());

    double tolerance = Constants.DriveTrain.errorTolerance;
    boolean atTargetPosition = ((flError <= tolerance) && (frError <= tolerance) && (blError <= tolerance) && (brError <= tolerance));
    return atTargetPosition;
  }

  public void toAutonomousMode() {
    double angle;
    if (IMUSubsystem.getYaw() >= 0) {
      angle = IMUSubsystem.getGyroRotation().getRadians();
    }else {
      angle = (2 * Math.PI) + IMUSubsystem.getGyroRotation().getRadians();
    }
    double xDif = 0.6604 * Math.cos(angle);
    double yDif = 0.6604 * Math.sin(angle);

    resetPose(pose.getX() + xDif, pose.getY() + yDif, pose.getRotation().getRadians());
  }

  public void toTeleopMode() {
    double angle;
    if (IMUSubsystem.getYaw() >= 0) {
      angle = IMUSubsystem.getGyroRotation().getRadians();
    }else {
      angle = (2 * Math.PI) + IMUSubsystem.getGyroRotation().getRadians();
    }
    double xDif = -0.6604 * Math.cos(angle);
    double yDif = -0.6604 * Math.sin(angle);

    resetPose(pose.getX() + xDif, pose.getY() + yDif, pose.getRotation().getRadians());
  }

  // public void driveCartesian (double X_speed, double Y_speed, double Z_rotation) {
  //   //++ I think this method is now redundant with PID stuff?
  //   speeds.driveCartesian(-Y_speed, X_speed, Z_rotation);
  // }
  public static Field2d getField() {
    return field;
  }



  @Override
  public void periodic() {

    xChange = IMUSubsystem.getXPosition() - xChange;
    yChange = IMUSubsystem.getYPosition() - yChange;

    MecanumDriveWheelSpeeds wheelspeeds = new MecanumDriveWheelSpeeds(
        frontLeftEncoder.getVelocity(),
        frontRightEncoder.getVelocity(),
        backLeftEncoder.getVelocity(),
        backRightEncoder.getVelocity()
    );
    speedErrorThreshold = speedErrorThresholdWidget.getEntry();
    // ~~ Use odometry object for calculating position
    ChassisSpeeds expectedSpeed = kinematics.toChassisSpeeds(wheelspeeds);
    ChassisSpeeds actualSpeed = new ChassisSpeeds(IMUSubsystem.getXVelocity(), IMUSubsystem.getYVelocity(), 0.0);

    Double xError = expectedSpeed.vxMetersPerSecond - actualSpeed.vxMetersPerSecond;
    Double yError = expectedSpeed.vyMetersPerSecond - actualSpeed.vyMetersPerSecond;
    
    Double speedError = Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));

    if (DriverStation.isTest()) {
      SmartDashboard.putNumber("Speed Error", speedError);
    }

    // ~~ Checks if the robot's wheels are slipping to determine if odometry or the imu would be more accurate
    if (speedError < speedErrorThreshold.getDouble(0.1)) {
      // ~~ Calculates position based on odometry
      pose = odometry.update(IMUSubsystem.getGyroRotation(), wheelspeeds);

      if (DriverStation.isTest()) {
        SmartDashboard.putBoolean("Slipping?", false);
      }
    } 
    else {
      // ~~ Calculates position based on imu
      Double newX = pose.getX() + xChange;
      Double newY = pose.getY() + yChange;
      Rotation2d newR = pose.getRotation();

      pose = new Pose2d(newX, newY, newR);

      // ~~ Updates odometry object with data from imu
      odometry.update(IMUSubsystem.getGyroRotation(), wheelspeeds);
      odometry.resetPosition(pose, IMUSubsystem.getGyroRotation());

      SmartDashboard.putBoolean("Slipping?", true);

    }

    SmartDashboard.putNumber("Robot x", pose.getX());
    SmartDashboard.putNumber("Robot y", pose.getY());
    SmartDashboard.putNumber("Robot rotation", pose.getRotation().getDegrees());

    SmartDashboard.putNumber("FL Speed", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("FL Position", frontLeftEncoder.getPosition());
    // ~~ Update field object for shuffleboard
    
    field.setRobotPose(pose);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}