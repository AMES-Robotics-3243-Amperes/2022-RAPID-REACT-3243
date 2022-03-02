// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climber_motor_R = new CANSparkMax(Constants.Climber.climber_motor_R, MotorType.kBrushless);
  private CANSparkMax climber_motor_L = new CANSparkMax(Constants.Climber.climber_motor_L, MotorType.kBrushless);
  private CANSparkMax grabber_R1 = new CANSparkMax(Constants.Climber.grabber_R1, MotorType.kBrushless);
  private CANSparkMax grabber_R2 = new CANSparkMax(Constants.Climber.grabber_R2, MotorType.kBrushless);
  private CANSparkMax grabber_L1 = new CANSparkMax(Constants.Climber.grabber_L1, MotorType.kBrushless);
  private CANSparkMax grabber_L2 = new CANSparkMax(Constants.Climber.grabber_L2, MotorType.kBrushless);

  private SparkMaxPIDController climber_motor_R_pid = climber_motor_R.getPIDController();
  private SparkMaxPIDController climber_motor_L_pid = climber_motor_L.getPIDController();
  private SparkMaxPIDController grabber_R1_pid = grabber_R1.getPIDController();
  private SparkMaxPIDController grabber_R2_pid = grabber_R2.getPIDController();
  private SparkMaxPIDController grabber_L1_pid = grabber_L1.getPIDController();
  private SparkMaxPIDController grabber_L2_pid = grabber_L2.getPIDController();

  public double climberAngle = 0;

  public ClimberSubsystem () {
  }

  public void actuateGrabber(CANSparkMax grabberpid, double to_angle){
    // :) actuate grabber motors
    
  }

  public void spinClimber(double speed){
    // :) spin climber motor
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
