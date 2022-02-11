// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.Constants; 

// ++ SparkMax & encoder imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import com.revrobotics.RelativeEncoder;




public class ShooterSubsystem extends SubsystemBase {

  // ++ spark max and encoder documentation: 
  // https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
  // https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax#getEncoder()
  // https://codedocs.revrobotics.com/java/com/revrobotics/relativeencoder
  

  // ++ make motor objects  
  private CANSparkMax flywheelMotor = new CANSparkMax( Constants.Shooter.flywheelMotorID, MotorType.kBrushless ); 
  private CANSparkMax hoodMotor = new CANSparkMax( Constants.Shooter.hoodMotorID, MotorType.kBrushless ); 

  // ++ make encoder objects (this might not be the right kind of encoder or even work)


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
