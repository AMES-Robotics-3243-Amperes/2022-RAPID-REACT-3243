// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climberMotorR = new CANSparkMax(Constants.Climber.climberMotorR, MotorType.kBrushless);
  private CANSparkMax climberMotorL = new CANSparkMax(Constants.Climber.climberMotorL, MotorType.kBrushless);
  private CANSparkMax grabberR0 = new CANSparkMax(Constants.Climber.grabberR0, MotorType.kBrushless);
  private CANSparkMax grabberR1 = new CANSparkMax(Constants.Climber.grabberR1, MotorType.kBrushless);
  private CANSparkMax grabberL0 = new CANSparkMax(Constants.Climber.grabberL0, MotorType.kBrushless);
  private CANSparkMax grabberL1 = new CANSparkMax(Constants.Climber.grabberL1, MotorType.kBrushless);

  private SparkMaxPIDController climberMotorRPID = climberMotorR.getPIDController();
  private SparkMaxPIDController climberMotorLPID = climberMotorL.getPIDController();
  private SparkMaxPIDController grabberR0PID = grabberR0.getPIDController();
  private SparkMaxPIDController grabberR1PID = grabberR1.getPIDController();
  private SparkMaxPIDController grabberL0PID = grabberL0.getPIDController();
  private SparkMaxPIDController grabberL1PID = grabberL1.getPIDController();

  private RelativeEncoder climberMotorREncoder = climberMotorR.getEncoder();
  private RelativeEncoder climberMotorLEncoder = climberMotorL.getEncoder();
  private RelativeEncoder grabberR0Encoder = grabberR0.getEncoder();
  private RelativeEncoder grabberR1Encoder = grabberR1.getEncoder();
  private RelativeEncoder grabberL0Encoder = grabberL0.getEncoder();
  private RelativeEncoder grabberL1Encoder = grabberL1.getEncoder();

  public double climberAngle = 0;
  public double grabberAngles[] = {0,0};

  private int grabberSoftCurrentLimit = 25;
  public int grabberHardCurrentLimit = 30;

  private int climberSoftCurrentLimit = 15;
  public int climberHardCurrentLimit = 20;

  public ClimberSubsystem () {
    // :) setting the soft current limits
    climberMotorL.setSmartCurrentLimit(climberSoftCurrentLimit);
    climberMotorR.setSmartCurrentLimit(climberSoftCurrentLimit);
    grabberL0.setSmartCurrentLimit(grabberSoftCurrentLimit);
    grabberL1.setSmartCurrentLimit(grabberSoftCurrentLimit);
    grabberR0.setSmartCurrentLimit(grabberSoftCurrentLimit);
    grabberR1.setSmartCurrentLimit(grabberSoftCurrentLimit);

    // :) setting the hard current limits
    climberMotorL.setSecondaryCurrentLimit(climberHardCurrentLimit);
    climberMotorR.setSecondaryCurrentLimit(climberHardCurrentLimit);
    grabberL0.setSecondaryCurrentLimit(grabberHardCurrentLimit);
    grabberL1.setSecondaryCurrentLimit(grabberHardCurrentLimit);
    grabberR0.setSecondaryCurrentLimit(grabberHardCurrentLimit);
    grabberR1.setSecondaryCurrentLimit(grabberHardCurrentLimit);

    // :) setting the P in the motor PIDs
    grabberL0PID.setP(0.8);
    grabberL1PID.setP(0.8);
    grabberR0PID.setP(0.8);
    grabberR1PID.setP(0.8);
    climberMotorLPID.setP(0.8);
    climberMotorRPID.setP(0.8);

    // :) resetting the encoder positions for the motors to 0
    climberMotorREncoder.setPosition(0);
    climberMotorLEncoder.setPosition(0);

    // :) left side is inverted
    grabberL0.setInverted(true);
    grabberL1.setInverted(true);
    grabberR0.setInverted(false);
    grabberR1.setInverted(false);
  }

  public void calibrateGrabbers(){
    spinGrabbers(0,1);
    spinGrabbers(1,1);
  }

  

  public void actuateGrabber(CANSparkMax grabberpid, double to_angle){
    // :) actuate grabber motors, to a specified angle
    
  }

  public void spinGrabbers(int side, double speed){
    // :) spin grabber motors at a speed, given which side of the climber you want to actuate (0 or 1). Try to spin around a speed of 0.2
    grabberAngles[side] += speed;
  }

  public void actuateClimber(double to_angle){
    // :) actuate both climber motors, to a specified angle

  }

  public void spinClimber(double speed){
    // :) spin climber both motors at a speed
    climberAngle += speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // :) update and spin the motors to their angles
    climberMotorLPID.setReference(climberAngle, ControlType.kPosition);
    climberMotorRPID.setReference(climberAngle, ControlType.kPosition);
    grabberL0PID.setReference(grabberAngles[0], ControlType.kPosition);
    grabberL1PID.setReference(grabberAngles[1], ControlType.kPosition);
    grabberR0PID.setReference(grabberAngles[0], ControlType.kPosition);
    grabberR1PID.setReference(grabberAngles[1], ControlType.kPosition);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
