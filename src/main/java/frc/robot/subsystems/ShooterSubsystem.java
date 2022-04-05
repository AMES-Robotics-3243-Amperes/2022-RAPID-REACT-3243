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

  

  // ++ declare encoder objects
  public RelativeEncoder flywheelEncoder;

  // ++ declare PID objects 
  private SparkMaxPIDController flywheelPID;

  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // ++ define encoder objects
    flywheelEncoder = flywheelMotor.getEncoder();

    // ++ define PID objects
    flywheelPID = flywheelMotor.getPIDController();



  }

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
    
    
    
    // This method will be called once per scheduler run


  }

}
