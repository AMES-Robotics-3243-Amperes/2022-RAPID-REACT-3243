// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Random;


/** ++ This class is used to read values from the LimeLight and do calculations based on those numbers. 
 *  Pretty much all of the math for calculating things (like flywheel speed, hood angle, etc) should be done HERE 
 * instead of the corresponding subsystem. (This is basically just a subsystem full of methods that get values)
 */
public class LimelightSubsystem extends SubsystemBase {

  // ++ table object
  static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  // ++ LimeLight values -- table entry objects
  static NetworkTableEntry tx = limelightTable.getEntry("tx");   // ++ x offset 
  static NetworkTableEntry ty = limelightTable.getEntry("ty");   // ++ y offset
  static NetworkTableEntry ta = limelightTable.getEntry("ta");   // ++ area of target 
  static NetworkTableEntry tv = limelightTable.getEntry("tv");   // 1 if limelight sees a valid target, 0 otherwise

  // ++ random class object to see if shuffleboard is updating properly
  Random Waffles = new Random(); 

  public boolean continueShooterRoutine = false;

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {


    
  }


  

  //  ++ ---------------- target position methods -----------

  /** ++ This method gets the X position of the target the Limelight sees. This is also the rotational error of the robot
   * @return the X position of the target (as an angle) */
  public double getTargetX() {
    return tx.getDouble(0.0);
  }

    /** ++ This method gets the Y position of the target the Limelight sees (including limelight angle offset) 
   * @return the Y position of the target (as an angle) */
  public static double getTargetY() {
    return (ty.getDouble(-Constants.Limelight.limelightAngleOffset) + Constants.Limelight.limelightAngleOffset);
    // ++ I did a weird default value to make the method return 0 if no value is found
  }

    /** ++ This method gets the area of the target the Limelight sees
   * @return the area position of the target (as a fraction of total camera area) */
  public double getTargetArea() {
    return ta.getDouble(0.0);
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


  // ++ ======= CALCULATION METHODS ===================================================
  // ++ these methods are used to calculate the necessary values for the ball's trajectory


  // ++ --------------- misc ----------------------------

  /** this finds the distance from the hub based on limelight values (y angle offset) */
  private static double findDistanceFromHub() {
    return (Constants.Limelight.shooterToHubHeight / Math.tan( getTargetY() ));
  }

  /** ++ this is the function that tells you the necessary flywheel velocity based on the distance. 
   * NOTE: this is NOT the implementation of this! This just converts from 
   */
  private static double flywheelVelocityFromDistance(double distance) {
    // ++ THIS IS JUST A PLACEHOLDER FOR NOW, WE'LL NEED TO FIND THE ACTUAL FUNCTION WHEN THE ROBOT WORKS
    return distance;
  }
  // ++ --------------- end misc  ------------------------





  // ++ --------------- trajectory stuff -------------------------------------

  /* ++   >>>>>>>>>>>> ***COOL DESMOS GRAPH*** https://www.desmos.com/calculator/jyxaoog9fd <<<<<<<<<<<<<<<<<<<<<<<<<<
  *
  * the graph should help give context for the calculations here.
  * I'm hoping the comments in the graph should be descriptive enough explain what's going on
  */



  /** ++ this finds the B coefficient of the trajectory parabola. look at the desmos graph for context
   * (desmos graph is in a comment above the definition of this function).
   * This method should really only be used to find the angle of the hood (below)
   * @param distanceFromHub this the distance between the robot and the center of the hub
   * @return the B coefficient
   */
  public double findBCoeff(double distanceFromHub) {
    double hubHeight = Constants.Limelight.shooterToHubHeight;
    double arbPointX = distanceFromHub + Constants.Limelight.arbPointXOffset;
    double arbPointY = hubHeight + Constants.Limelight.arbPointYOffset;

    double bCoeff = ( ( (-(Math.pow(distanceFromHub, 2) * arbPointY)) + (Math.pow(arbPointX, 2) * hubHeight) ) 
    / ( (Math.pow(arbPointX, 2) * distanceFromHub) - (Math.pow(distanceFromHub, 2) * arbPointX) ) 
    ); // ++ this is what happens when you do a linear algebra in code

    return bCoeff;
  }
  
  /** ++ this method returns angle the hood should be to make the ball in the hub*/
  public double findTargetHoodAngle() {
    return Math.atan( findBCoeff( findDistanceFromHub() ) );
  }

  /** ++ this method gives you the necessary velocity of the flywheel  */
  public static double giveTargetFlywheelVelocity() {
    return flywheelVelocityFromDistance( findDistanceFromHub() );
  }
  // ++ ----------- end trajectory stuff ---------------------------------------


  // ++ ======== END CALCULATION METHODS ==============================================


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
