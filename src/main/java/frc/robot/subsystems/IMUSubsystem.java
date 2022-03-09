// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class IMUSubsystem extends SubsystemBase {

  // ~~ IMU object - gyro and accelerometer
  AHRS imu = new AHRS();

  // ~~ table object
  NetworkTable imuTable = NetworkTableInstance.getDefault().getTable("limelight");
  
  // ~~ LimeLight values -- table entry objects
  NetworkTableEntry tYaw = imuTable.getEntry("yaw");                // ~~ yaw - rotation around vertical axis
  NetworkTableEntry tPitch = imuTable.getEntry("pitch");            // ~~ pitch - rotation around left/right axis
  NetworkTableEntry tRoll = imuTable.getEntry("roll");              // ~~ roll - rotation around front/back axis

  NetworkTableEntry tXVelocity = imuTable.getEntry("x velocity");   // ~~ x velocity
  NetworkTableEntry tYVelocity = imuTable.getEntry("y velocity");   // ~~ y velocity
  NetworkTableEntry tZVelocity = imuTable.getEntry("z velocity");   // ~~ z velocity

  NetworkTableEntry tXPos = imuTable.getEntry("x offset");          // ~~ x offset
  NetworkTableEntry tYPos = imuTable.getEntry("y offset");          // ~~ y offset
  NetworkTableEntry tZPos = imuTable.getEntry("z offset");          // ~~ z offset

  
  /** Creates a new IMUSubsystem. */
  public IMUSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tYaw.setDouble(imu.getYaw());
    tPitch.setDouble(imu.getPitch());
    tRoll.setDouble(imu.getRoll());
    tXVelocity.setDouble(imu.getVelocityX());
    tYVelocity.setDouble(imu.getVelocityY());
    tZVelocity.setDouble(imu.getVelocityZ());
    tXPos.setDouble(imu.getDisplacementX());
    tYPos.setDouble(imu.getDisplacementY());
    tZPos.setDouble(imu.getDisplacementZ());
  }
}
