// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShuffleboardSubsystem;

import frc.robot.Constants;

// ++ SparkMax & encoder imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;



public class ShooterSubsystem extends SubsystemBase {

  // ++ spark max and encoder documentation: 
  // https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
  // https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax#getEncoder()
  // https://codedocs.revrobotics.com/java/com/revrobotics/relativeencoder
  

  // ++ make motor objects  
  private CANSparkMax flywheelMotor = new CANSparkMax( Constants.Shooter.flywheelMotorID, MotorType.kBrushless ); 
    // ++ the servo model we're using is the HS-805BB (by HITEC)
  private Servo hoodMotor = new Servo( Constants.Shooter.hoodServoPWMID);
  

  // ++ declare encoder objects
  public RelativeEncoder flywheelEncoder;

  // ++ declare PID objects 
  private SparkMaxPIDController flywheelPID;

  private double hoodAngleTarget;


  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // ++ define encoder objects
    flywheelEncoder = flywheelMotor.getEncoder();

    // ++ define PID objects
    flywheelPID = flywheelMotor.getPIDController();



  }

  // ++ ============== HOOD STUFF ===================================


  /** ++ this sets the position of the hood. USE THIS METHOD instead of just setting it; it has some protections
   * to help prevent the code from crashing.
   */
  public void setServoPositionFromHoodAngle(double hoodAngleTarget) {
    double servoAngleTarget = servoAngleFromHoodAngle( hoodAngleTarget );
    double servoPositionTarget = servoPositionFromServoAngle( servoAngleTarget );


    // ++ this if statement should be the last thing done before the .set() to help the code not crash
    if (servoPositionTarget > 1.0) {
      servoPositionTarget = 1.0;
    } else if (servoPositionTarget < 0.0){
      servoPositionTarget = 0.0;
    }

    hoodMotor.set( servoPositionTarget );

  }
  
  /** ++ this gives you the necessary servo angle based on the desired angle of the hood
   * @param hoodAngle this is the desired angle of the hood
   * @return servoAngle this is the necessary angle of the servo to achieve that hood angle
   */
  public double servoAngleFromHoodAngle(double hoodAngle) {
    double servoAngle;
    /* ++ the values for this if statement are HARD-CODED because the code crashes if the equation below ONLY
    * works for angles ( 26 <= a <= 42 ), otherwise the code crashes! DONT CHANGE THEM! */
    if (hoodAngle <= 26){
      hoodAngle = 26;
    } else if (hoodAngle >= 42){
      hoodAngle = 42;
    }
    servoAngle = Math.toDegrees( (Math.asin( (( Math.toRadians(hoodAngle) - 0.596)/0.154 )) + 1.683 ) / 1.105  ) + 1.0;
    return ( servoAngle );
  }

  /** ++ converts from a servo angle (out of 180) to a position of the servo on [0, 1] */
  public double servoPositionFromServoAngle(double servoAngle) {
    return (servoAngle / Constants.Shooter.maxSpecHoodServoAngle);
  }


  /** ++ this returns the last position the servo was told to set to */
  public double getPrevServoPosition() {
    return hoodMotor.get();
  }
  // ++ ---- calculation methods



  // /** ++ converts from a position of the servo on [0, 1] to a servo angle (out of 180) */
  // public double servoPositionToServoAngle(double position) {
  //   return (position * Constants.Shooter.maxSpecHoodServoAngle);
  // }





  // ++ ============== END HOOD STUFF ===============================
// maya #2 is cooler than u



  // ++ ============ FLYWHEEL STUFF ==============================

  /** ++ this is the main method that should be used to set the flywheel speed */
  public void setFlywheelSpeed(double speed) {
    flywheelPID.setReference(speed, ControlType.kVelocity);
  }

  public void stopFlywheel(){
    flywheelMotor.set(0.0);
  }

  /** ++ returns the actual velocity of the flywheel */
  public double getCurrentFlywheelSpeed() {
    // ++ returns the velocity of the flywheelEncoder
    return flywheelEncoder.getVelocity();
  }

  /** this sets the P, I, and D values for the flywheel */
  public void setFlyhweelPIDValues() {
    double values[] = ShuffleboardSubsystem.getFlywheelPIDValues();
    flywheelPID.setP( values[0] ); 
    flywheelPID.setI( values[1] );
    flywheelPID.setD( values[2] );
    flywheelPID.setFF( values[3] );
  }
  // ++ ============ END FLYWHEEL STUFF ==========================


  @Override
  public void periodic() {
    
    
    // ShuffleboardSubsystem.displayFlywheelSpeed( getCurrentFlywheelSpeed() );
    ShuffleboardSubsystem.displayPrevServoSet( getPrevServoPosition() );
    
    // This method will be called once per scheduler run


  }

}
