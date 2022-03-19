// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class ClimberSubsystem extends SubsystemBase {
  // :) all the object initialization

  private CANSparkMax climberMotorR = new CANSparkMax(Constants.Climber.climberMotorR, MotorType.kBrushless);
  private CANSparkMax climberMotorL = new CANSparkMax(Constants.Climber.climberMotorL, MotorType.kBrushless);
  private CANSparkMax grabberR0 = new CANSparkMax(Constants.Climber.grabberR0, MotorType.kBrushless);
  private CANSparkMax grabberR1 = new CANSparkMax(Constants.Climber.grabberR1, MotorType.kBrushless);
  private CANSparkMax grabberL0 = new CANSparkMax(Constants.Climber.grabberL0, MotorType.kBrushless);
  private CANSparkMax grabberL1 = new CANSparkMax(Constants.Climber.grabberL1, MotorType.kBrushless);

  private ShuffleboardTab shuffleboard_tab = Shuffleboard.getTab("Climber");

  //getting motor encoders, pids, etc.

  private SparkMaxPIDController climberMotorRPID = climberMotorR.getPIDController();
  private SparkMaxPIDController climberMotorLPID = climberMotorL.getPIDController();
  private SparkMaxPIDController grabberR0PID = grabberR0.getPIDController();
  private SparkMaxPIDController grabberR1PID = grabberR1.getPIDController();
  private SparkMaxPIDController grabberL0PID = grabberL0.getPIDController();
  private SparkMaxPIDController grabberL1PID = grabberL1.getPIDController();

  public RelativeEncoder climberMotorREncoder = climberMotorR.getEncoder();
  public RelativeEncoder climberMotorLEncoder = climberMotorL.getEncoder();
  private RelativeEncoder grabberR0Encoder = grabberR0.getEncoder();
  private RelativeEncoder grabberR1Encoder = grabberR1.getEncoder();
  private RelativeEncoder grabberL0Encoder = grabberL0.getEncoder();
  private RelativeEncoder grabberL1Encoder = grabberL1.getEncoder();

  // :) variables

  public boolean isCalibrated = false;
  public boolean isCalibrating = false;
  public boolean isGrabberCalibrated[] = {false,false,false,false};

  public double climberAngle = 0;
  public double grabberAngles[] = {0,0};

  private final double calibrationCurrent = 12;

  private final int grabberSoftCurrentLimit = 15;
  public final int grabberHardCurrentLimit = 20;

  private final int climberSoftCurrentLimit = 80;
  public final int climberHardCurrentLimit = 80;

  private final double maxTemp = 40;
  public boolean isTooHot = false;

  private final double gripperOpenMaximum = 67;
  private final double gripperClosedMinimum = 0.4;

  public ClimberSubsystem () {
    // :) reset things
    climberAngle = 0;

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

    // :) setting the I in the motor PIDs
    //climberMotorLPID.setI(0.001);
    //climberMotorRPID.setI(0.001);

    // :) resetting the encoder positions for the motors to 0
    climberMotorREncoder.setPosition(0);
    climberMotorLEncoder.setPosition(0);

    // :) left side is inverted??
    grabberL0.setInverted(true);
    grabberL1.setInverted(true);
    grabberR0.setInverted(true);
    grabberR1.setInverted(true);


    // :) set motors to follow
    
    // :) this was making the follower motor not follow with the full strength of the motor so I removed it temporarily
    //climberMotorL.follow(climberMotorR,true);
    //grabberL1.follow(grabberR1,false);
    //grabberL0.follow(grabberR0,false);

    // :) set motor coast/brake
    climberMotorL.setIdleMode(IdleMode.kCoast);
    climberMotorR.setIdleMode(IdleMode.kCoast);
    grabberL1.setIdleMode(IdleMode.kBrake);
    grabberR1.setIdleMode(IdleMode.kBrake);
    grabberL0.setIdleMode(IdleMode.kBrake);
    grabberR0.setIdleMode(IdleMode.kBrake);

  }

  public void initialize(){
    
    
  }

  public void resetMotorPosReadings(){
    // :) resetting the encoder positions for the motors to 0
    climberMotorREncoder.setPosition(0);
    climberMotorLEncoder.setPosition(0);

    grabberR1Encoder.setPosition(0);
    grabberL1Encoder.setPosition(0);
    grabberR0Encoder.setPosition(0);
    grabberL0Encoder.setPosition(0);

    climberAngle = 0;
    grabberAngles[0]=0;
    grabberAngles[1]=0;
  }

  public void calibrateGrabbers(){
    isCalibrated = false;

    for (int i=0; i<isGrabberCalibrated.length; i++) {
      isGrabberCalibrated[i] = false;
    }
    
    grabberL0.set(-0.2); // negative? yes
    grabberR0.set(-0.2);
    grabberL1.set(-0.2);
    grabberR1.set(-0.2);

    isCalibrating = true;
  }


  public void actuateGrabber(int side, double revolutions){
    // :) actuate grabber motors, to a specified number of revolutions from starting angle
    grabberAngles[side] = revolutions;
  }

  public void spinGrabbers(int side, double speed){
    // :) spin grabber motors at a speed, given which side of the climber you want to actuate (0 or 1). Try to spin around a speed of 0.2
    if (grabberAngles[side]+speed > gripperClosedMinimum && grabberAngles[side]+speed < gripperOpenMaximum) {
      grabberAngles[side] += speed;
    }
  }

  public void actuateClimber(double revolutions){
    // :) actuate both climber motors, to a specified number of revolutions from starting angle
    climberAngle = revolutions;
  }

  public void spinClimber(double speed){
    // :) spin climber both motors at a speed
    climberAngle += speed;
    //climberMotorR.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
    // :) update and spin the motors to their angles

    // :) super long if-statements are to prevent the motors from overrunning too far. (hopefully they work...)
    //if ( (climberAngle-climberMotorREncoder.getPosition()>1 && climberAngle<climberMotorREncoder.getPosition()) || (climberAngle-climberMotorREncoder.getPosition()<-0.1 && climberAngle>climberMotorREncoder.getPosition()) ) {
      climberMotorRPID.setReference(climberAngle, ControlType.kPosition);
      climberMotorLPID.setReference(climberAngle, ControlType.kPosition);
    //}

    if (isCalibrated){
      if ((grabberL0Encoder.getPosition()<gripperOpenMaximum+2 && grabberL0Encoder.getPosition()<grabberAngles[0]) ||
          (grabberL0Encoder.getPosition()>gripperClosedMinimum && grabberL0Encoder.getPosition()>grabberAngles[0]) ) {
      //   grabberL0PID.setReference(grabberAngles[0], ControlType.kPosition);
      }
      if ((grabberR0Encoder.getPosition()<gripperOpenMaximum+2 && grabberR0Encoder.getPosition()<grabberAngles[0]) ||
          (grabberR0Encoder.getPosition()>gripperClosedMinimum && grabberR0Encoder.getPosition()>grabberAngles[0])) {
      //   grabberR0PID.setReference(grabberAngles[0], ControlType.kPosition);
            SmartDashboard.putBoolean("moving R0", true);
      } else {
        SmartDashboard.putBoolean("moving R0", false);
      }
      if ((grabberL1Encoder.getPosition()<gripperOpenMaximum+2 && grabberL1Encoder.getPosition()<grabberAngles[1]) ||
          (grabberL1Encoder.getPosition()>gripperClosedMinimum && grabberL1Encoder.getPosition()>grabberAngles[1])) {
      //   grabberL1PID.setReference(grabberAngles[1], ControlType.kPosition);
      }
      if ((grabberR1Encoder.getPosition()<gripperOpenMaximum+2 && grabberR1Encoder.getPosition()<grabberAngles[1]) ||
          (grabberR1Encoder.getPosition()>gripperClosedMinimum && grabberR1Encoder.getPosition()>grabberAngles[1])) {
      //   grabberR1PID.setReference(grabberAngles[1], ControlType.kPosition);
      }
    } else if (isCalibrating){
      if (grabberR0.getOutputCurrent() > calibrationCurrent) {
        grabberR0.set(0);
        grabberR0Encoder.setPosition(0);
        grabberAngles[0]=0;
        isGrabberCalibrated[0] = true;
      }
      // :) remove when not testing!!!!!!!!!!!!!
      isGrabberCalibrated[1] = true;
      if (grabberL0.getOutputCurrent() > calibrationCurrent){
        grabberL0.set(0);
        grabberL0Encoder.setPosition(0);
        grabberAngles[0]=0;
        isGrabberCalibrated[1] = true;
      }
      if (grabberR1.getOutputCurrent() > calibrationCurrent) {
        grabberR1.set(0);
        grabberR0Encoder.setPosition(0);
        grabberAngles[1]=0;
        isGrabberCalibrated[2] = true;
      }
      if (grabberL1.getOutputCurrent() > calibrationCurrent){
        grabberL1.set(0);
        grabberL0Encoder.setPosition(0);
        grabberAngles[1]=0;
        isGrabberCalibrated[3] = true;
      }

      // :) set isCalibrated to true only if all in the list is set to true
      isCalibrated = isGrabberCalibrated[0] && isGrabberCalibrated[1] && isGrabberCalibrated[2] && isGrabberCalibrated[3];
      // :) also stop calibrating when the above conditions are met
      isCalibrating = !isCalibrated;
      
    }
    

    // :) putting motors stuff on smartdashboard
    SmartDashboard.putNumber("Avg. Temp of grabber motors 0 (Celcius)", (grabberL0.getMotorTemperature()+grabberR0.getMotorTemperature())/2);
    SmartDashboard.putNumber("Avg. Temp of grabber motors 1 (Celcius)", (grabberL1.getMotorTemperature()+grabberR1.getMotorTemperature())/2);
    SmartDashboard.putNumber("Avg. Temp of climber motors (Celcius)", (climberMotorL.getMotorTemperature()+climberMotorR.getMotorTemperature())/2);
    SmartDashboard.putNumber("Grabber side 0", grabberAngles[0]);
    SmartDashboard.putNumber("Grabber side 1", grabberAngles[1]);
    SmartDashboard.putNumber("Grabber side 0 encoder", grabberR0Encoder.getPosition());
    SmartDashboard.putBooleanArray("isCalibratedArray", isGrabberCalibrated);
    SmartDashboard.putBoolean("isCalibrated", isCalibrated);
    
    // SmartDashboard.putNumber("Current of climber L", climberMotorL.getOutputCurrent());
    // SmartDashboard.putNumber("Current of climber R", climberMotorR.getOutputCurrent());
    // SmartDashboard.putNumber("Shaktool angle", climberMotorLEncoder.getPosition());
    if (climberMotorR.getMotorTemperature()>maxTemp || climberMotorL.getMotorTemperature()>maxTemp || grabberL0.getMotorTemperature()>maxTemp || grabberL1.getMotorTemperature()>maxTemp || grabberR0.getMotorTemperature()>maxTemp || grabberR1.getMotorTemperature()>maxTemp) {
      isTooHot = true;
    } else {
      isTooHot = false;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
