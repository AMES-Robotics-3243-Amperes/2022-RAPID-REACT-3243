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


/** ++ This class is used to read values from the LimeLight and do calculations based on those numbers. 
 *  Pretty much all of the math for calculating things (like flywheel speed, hood angle, etc) should be done HERE 
 * instead of the corresponding subsystem. 
 */
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


  

  //  ++ ---------------- target position methods -----------

  /** ++ This method gets the X position of the target the Limelight sees
   * @param defaultReturn this is the default value for the method to return if there isn't any Limelight value.
   * This should probably be 0, but I still put it as a param just incase it's necessary. 
   * @return the X position of the target (as an angle) */
  public double getTargetX(double defaultReturn) {
    return tx.getDouble(defaultReturn);
  }

    /** ++ This method gets the Y position of the target the Limelight sees
   * @param defaultReturn this is the default value for the method to return if there isn't any Limelight value.
   * This should probably be 0, but I still put it as a param just incase it's necessary. 
   * @return the Y position of the target (as an angle) */
  public double getTargetY(double defaultReturn) {
    return ty.getDouble(defaultReturn);
  }

    /** ++ This method gets the area of the target the Limelight sees
   * @param defaultReturn this is the default value for the method to return if there isn't any Limelight value.
   * This should probably be 0, but I still put it as a param just incase it's necessary. 
   * @return the area position of the target (as a fraction of total camera area) */
  public double getTargetArea(double defaultReturn) {
    return ta.getDouble(defaultReturn);
  }

  /** ++ this method determines if the Limelight sees any valid targets
   * @return true if it sees one or more validtargets, false otherwise
   */
  public Boolean isTargetValid() {
    // ++ NOTE: "ta" returns "1.0" if it sees ANY number of valid targets
      // ++ for example, it would still return "1.0" if it sees 3 valid targets
    double tvOutput = tv.getDouble(0.0);
    if (tvOutput == 1.0){
      return true;
    } else {
      return false;
    }
  }

  // ++ --------------------------------------------------------------------------------




  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
