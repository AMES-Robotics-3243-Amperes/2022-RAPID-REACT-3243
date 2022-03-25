// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class IMUSubsystem extends SubsystemBase {

  // ~~ IMU object - gyro and accelerometer
  private static final AHRS imu = new AHRS();

  
  /** Creates a new IMUSubsystem. */
  public IMUSubsystem() {
    

  }

  public static double getYaw() {
    return imu.getYaw();
    // return 0.0;
  }

  public static double getPitch() {
    return imu.getPitch();
  }

  public static double getRoll() {
    return imu.getRoll();
  }

  public static double getXVelocity() {
    return imu.getVelocityX();
  }

  public static double getYVelocity() {
    return imu.getVelocityY();
  }

  public static double getZVelocity() {
    return imu.getVelocityZ();
  }

  public static double getXPosition() {
    return imu.getDisplacementX();
  }

  public static double getYPosition() {
    return imu.getDisplacementY();
  }

  public static double getZPosition() {
    return imu.getDisplacementZ();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // tYaw.setDouble(imu.getYaw());
    // tPitch.setDouble(imu.getPitch());
    // tRoll.setDouble(imu.getRoll());
    // tXVelocity.setDouble(imu.getVelocityX());
    // tYVelocity.setDouble(imu.getVelocityY());
    
    // tZVelocity.setDouble(imu.getVelocityZ());
    // tXPos.setDouble(imu.getDisplacementX());
    // tYPos.setDouble(imu.getDisplacementY());
    // tZPos.setDouble(imu.getDisplacementZ());
  }
}
