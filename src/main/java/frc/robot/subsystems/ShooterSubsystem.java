// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

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
  private CANSparkMax hoodMotor = new CANSparkMax( Constants.Shooter.hoodMotorID, MotorType.kBrushless ); 
  private Servo servoMotor = new Servo( 0 );

  // ++ declare encoder objects
  public RelativeEncoder hoodEncoder; 
  public RelativeEncoder flywheelEncoder;
  public RelativeEncoder servoEncoder;

  // ++ declare PID objects
  private SparkMaxPIDController hoodPID;
  private SparkMaxPIDController flywheelPID;


  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    // ++ define encoder objects
    flywheelEncoder = flywheelMotor.getEncoder();
    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPositionConversionFactor(Constants.Shooter.motorToHoodAngle);

    // ++ define PID objects
    flywheelPID = flywheelMotor.getPIDController();
    hoodPID = hoodMotor.getPIDController();

  }

  // ++ ============== HOOD STUFF ===================================

  public void setHoodAngle(double angle) {
    hoodPID.setReference(angle, ControlType.kPosition);
  }

  public void setServoAngle(double servoAngle) {
    servoMotor.setAngle(servoAngle);
  }

  public double getHoodAngle(){
    return hoodEncoder.getPosition();
 }

  /** this sets the P, I, and D values for the hood */
  public void setHoodPIDValues(){
    double pGain = Constants.Shooter.hoodPGain;
    double iGain = Constants.Shooter.hoodIGain;
    double dGain = Constants.Shooter.hoodDGain;
    hoodPID.setP(pGain);
    hoodPID.setI(iGain);
    hoodPID.setD(dGain);
  }


  // ++ ============== END HOOD STUFF ===============================
// maya #2 is cooler than u



  // ++ ============ FLYWHEEL STUFF ==============================

  public void setFlywheelSpeed(double speed) {
    // flywheelPID.setReference(speed, ControlType.kVelocity);
    flywheelMotor.set(speed);
  }

  public void stopFlywheel(){
    flywheelMotor.set(0.0);
  }

  public double getFlywheelSpeed() {
    // ++ returns the velocity of the flywheelEncoder
    return flywheelEncoder.getVelocity();
  }

  /** this sets the P, I, and D values for the flywheel */
  public void setFlyhweelPIDValues() {
    double pGain = Constants.Shooter.flywheelPGain; //SmartDashboard.getNumber("shootP", Constants.Shooter.flywheelPGain);
    double iGain = Constants.Shooter.flywheelIGain; //SmartDashboard.getNumber("shootI", Constants.Shooter.flywheelIGain);
    double dGain = Constants.Shooter.flywheelDGain; //SmartDashboard.getNumber("shootP", Constants.Shooter.flywheelDGain);
    flywheelPID.setP(pGain); 
    flywheelPID.setI(iGain);
    flywheelPID.setD(dGain);
  }
  // ++ ============ END FLYWHEEL STUFF ==========================


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

}
