// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Random;

public class LimelightSubsystem extends SubsystemBase {

  // ++ table object
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  // ++ LimeLight values -- table entry objects
  NetworkTableEntry tx = limelightTable.getEntry("tx");   // ++ x offset 
  NetworkTableEntry ty = limelightTable.getEntry("ty");   // ++ y offset
  NetworkTableEntry ta = limelightTable.getEntry("ta");   // ++ area of target 
  NetworkTableEntry tv = limelightTable.getEntry("tv");   // 1 if limelight sees a valid target, 0 otherwise

  // ++ random class object to see if shuffleboard is updating properly
  Random Waffles = new Random(); 


  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {


    
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

      // ++ periodically reads limelight 
      double targetXPos = tx.getDouble(0.0); 
      double targetYPos = ty.getDouble(0.0); 
      double targetArea = ta.getDouble(0.0);
      double targetIsValid = tv.getDouble(9.0);   

      // ++ puts the read values to shuffleboard
      SmartDashboard.putNumber("x position", targetXPos);
      SmartDashboard.putNumber("y position", targetYPos);
      SmartDashboard.putNumber("target area", targetArea); 
      SmartDashboard.putNumber("valid target?", targetIsValid); 

      SmartDashboard.putNumber("test field", Waffles.nextInt(100));




  }
}
