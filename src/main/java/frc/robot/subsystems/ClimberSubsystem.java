// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation;

public class ClimberSubsystem extends SubsystemBase {
  // :) all the object initialization

  private CANSparkMax climberMotorR = new CANSparkMax(Constants.Climber.climberMotorR, MotorType.kBrushless);
  private CANSparkMax climberMotorL = new CANSparkMax(Constants.Climber.climberMotorL, MotorType.kBrushless);
  private CANSparkMax grabberR0 = new CANSparkMax(Constants.Climber.grabberR0, MotorType.kBrushless);
  private CANSparkMax grabberR1 = new CANSparkMax(Constants.Climber.grabberR1, MotorType.kBrushless);
  private CANSparkMax grabberL0 = new CANSparkMax(Constants.Climber.grabberL0, MotorType.kBrushless);
  private CANSparkMax grabberL1 = new CANSparkMax(Constants.Climber.grabberL1, MotorType.kBrushless);
  private Servo pawlR0 = new Servo(Constants.Climber.pawlR0);
  private Servo pawlR1 = new Servo(Constants.Climber.pawlR1);
  private Servo pawlL0 = new Servo(Constants.Climber.pawlL0);
  private Servo pawlL1 = new Servo(Constants.Climber.pawlL1);

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

  public double spinFF = 0;
  public double spinI=0;
  public double spinP=0;
  public double spinD=0;


  public int currentClimberStep = 0;
  public int previousClimberStep = 0;
  public boolean isRunningClimbCommand = false;
  public boolean isClimberStepStopped = false;
  public boolean prevStopped = false;


  public boolean isCalibrated = false;
  public boolean isCalibrating = false;
  public boolean isGrabberCalibrated[] = {false,false,false,false};
  public boolean isPawlsConnected;

  public double climberAngle = 0;
  public double grabberAngles[] = {0,0};
  public double encoderGrabberAngles[]  = {0,0}; //[0] is side 0, [1] is side 1
  public double prevEncoderGrabberAngles[] = {0,0};
  public double encoderClimberAngle;
  public double climberAngleDegrees = 0;
  public double pawlServoAngles[] = {Constants.Climber.pawlOpen,Constants.Climber.pawlOpen};

  public double avgSpinnerCurrentDraw;

  private double calibrationCurrent = 7;

  private final int grabberSoftCurrentLimit = 15;
  public final int grabberHardCurrentLimit = 20;

  private final int climberSoftCurrentLimit = 25;
  public final int climberHardCurrentLimit = 30;

  private final double maxTemp = 40;
  public boolean isTooHot = false;

  public final double gripperOpenMaximum = 67;
  public final double gripperClosedMinimum = 0.4;


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
    grabberL0PID.setP(0.025);  //all of these work okay at 0.03 but lets try a different thing for now
    grabberL1PID.setP(0.025);
    grabberR0PID.setP(0.025);
    grabberR1PID.setP(0.025);


    // :) setting the output range in the motor PIDs
    // grabberL0PID.setOutputRange(-0.5, 0.5); // :) delete maybe? maybe not we'll see
    // grabberL1PID.setOutputRange(-0.5, 0.5);
    // grabberR0PID.setOutputRange(-0.5, 0.5);
    // grabberR1PID.setOutputRange(-0.5, 0.5);
    climberMotorLPID.setOutputRange(-0.35, 0.35);
    climberMotorRPID.setOutputRange(-0.35, 0.35);

    // :) resetting the encoder positions for the motors to 0
    climberMotorREncoder.setPosition(0);
    climberMotorLEncoder.setPosition(0);

    // :) left side is inverted??
    grabberL0.setInverted(true);
    grabberL1.setInverted(true);
    grabberR0.setInverted(true);
    grabberR1.setInverted(true);
    climberMotorL.setInverted(false);
    climberMotorR.setInverted(true);

    
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
    SmartDashboard.putNumber("calibration speed", -0.17);
    SmartDashboard.putNumber("calibration current", 20);
  }

  public void initialize(){
    climberAngle=0;
    climberMotorLEncoder.setPosition(0);
    climberMotorREncoder.setPosition(0);
    currentClimberStep = 0;
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


    //climberMotorRPID.setReference(0,ControlType.kPosition);
    //climberMotorLPID.setReference(0,ControlType.kPosition);
  }

  public void calibrateGrabbers(){
    isCalibrated = false;

    for (int i=0; i<isGrabberCalibrated.length; i++) {
      isGrabberCalibrated[i] = false;
    }
    
    grabberL0.set(SmartDashboard.getNumber("calibration speed", -0.17)); // negative? yes
    grabberR0.set(SmartDashboard.getNumber("calibration speed", -0.17));
    grabberL1.set(SmartDashboard.getNumber("calibration speed", -0.17));
    grabberR1.set(SmartDashboard.getNumber("calibration speed", -0.17)); //-0.1? no

    isCalibrating = true;
  }


  public void actuateGrabber(int side, double revolutions){
    // :) actuate grabber motors, to a specified number of revolutions from closed angle (as long as it calibrates and is within motor range after calibration)
    grabberAngles[side] = Math.max(gripperClosedMinimum, Math.min(revolutions, gripperOpenMaximum));
  }            // above basically says: {gripperClosedMinimum<revolutions<gripperOpenMaximum} which limits it between the two numbers.

  public void spinGrabbers(int side, double speed){
    // :) spin grabber motors at a speed, given which side of the climber you want to actuate (0 or 1). Try to spin around a speed of 0.2
    if (grabberAngles[side]+speed >= gripperClosedMinimum && grabberAngles[side]+speed <= gripperOpenMaximum) {
      grabberAngles[side] += speed;
    }
  }


  // :) climber spinner is on a 64:1 gearbox which connects to a 15-tooth sprocket that is connected by chain to a 54-tooth sprocket
  // :) so unless my math is wrong (pretty likely haha) the ratio from the motor angle to the climber angle should be 64*(54/15):1 (which equals 230.4:1 and Milo got the same number)
  // :) then divide above number by 360 to set motor in terms of degrees
  // :') ok yeah my math was wrong
  public void actuateClimber(double revolutions){
    // :) actuate both climber motors, to a specified number of revolutions from starting angle
    climberAngle = revolutions;
    climberMotorRPID.setReference(climberAngle, ControlType.kPosition);
    climberMotorLPID.setReference(climberAngle, ControlType.kPosition);
  }

  public void spinClimber(double speed){
    // :) spin climber both motors at a speed, probably a bad idea to use this... here it is in case
    climberAngle += speed;
    //climberMotorR.set(speed);
  }

  @Override
  public void periodic() {



    SmartDashboard.getNumber("spin FF", 0);
    SmartDashboard.getNumber("spin I", 0);
    SmartDashboard.getNumber("spin P", 0.025);
    SmartDashboard.getNumber("spin d", 0);

    // :) stuff is commented out cos we don't really need to tune these PIDs anymore
    //if (spinFF!=SmartDashboard.getNumber("spin FF", 0) || spinI!=SmartDashboard.getNumber("spin I", 0) || spinP!=SmartDashboard.getNumber("spin P", 0.025) || spinD!=SmartDashboard.getNumber("spin D", 0)) {
      spinFF = 0; //SmartDashboard.getNumber("spin FF", 0);
      spinI = 0.000001; //SmartDashboard.getNumber("spin I", 0);
      spinP = 0.15; //SmartDashboard.getNumber("spin P", 0.025); //0.15 for kP, 0.000001 for kI, and 0 for everything else with currently set output range works pretty well so far, gotta test tho.
      spinD = 0; //SmartDashboard.getNumber("spin D", 0);
      // move this back up to the pid stuff later
      climberMotorLPID.setP(spinP);
      climberMotorRPID.setP(spinP);
      // :) setting the I in the motor PIDs
      climberMotorLPID.setI(spinI);
      climberMotorRPID.setI(spinI);
      // :) setting the FF in the motor PIDs
      climberMotorLPID.setFF(spinFF);
      climberMotorRPID.setFF(spinFF);

      climberMotorLPID.setD(spinD);
      climberMotorRPID.setD(spinD);
    //}
    calibrationCurrent=SmartDashboard.getNumber("calibration current", 20);

    encoderGrabberAngles[0] = (grabberL0Encoder.getPosition()+grabberR0Encoder.getPosition())/2; //is average
    encoderGrabberAngles[1] = (grabberL1Encoder.getPosition()+grabberR1Encoder.getPosition())/2;
    encoderClimberAngle = (climberMotorLEncoder.getPosition()+climberMotorREncoder.getPosition())/2; //also is average
    
    avgSpinnerCurrentDraw = (climberMotorL.getOutputCurrent()+climberMotorR.getOutputCurrent())/2;

    climberAngleDegrees = climberAngle*(230.4/360); // :) don't trust this number.....

    pawlL0.setAngle(pawlServoAngles[0]);
    pawlR0.setAngle(pawlServoAngles[0]);
    pawlL1.setAngle(pawlServoAngles[1]);
    pawlR1.setAngle(pawlServoAngles[1]);

    

    // This method will be called once per scheduler run

    
    // :) update and spin the motors to their angles

    // :) super long if-statement is to prevent the motors from overrunning too far. (hopefully it works...) 
    // :') spoiler alert: it didn't work
    //if ( (climberAngle-climberMotorREncoder.getPosition()>1 && climberAngle<climberMotorREncoder.getPosition()) || (climberAngle-climberMotorREncoder.getPosition()<-0.1 && climberAngle>climberMotorREncoder.getPosition()) ) {
      
    //}

    if (DriverStation.getMatchTime()<1.5 && DriverStation.isTeleop()){
      // :) engage pawls
      pawlServoAngles[0] = Constants.Climber.pawlClosed;
      pawlServoAngles[1] = Constants.Climber.pawlClosed;
    }
    if (currentClimberStep == 7){
      pawlServoAngles[0] = Constants.Climber.pawlClosed;
    }

    if (prevEncoderGrabberAngles[0] < encoderGrabberAngles[0] && pawlServoAngles[0]==Constants.Climber.pawlClosed){
      grabberAngles[0]=prevEncoderGrabberAngles[0];
    }
    if (prevEncoderGrabberAngles[1] < encoderGrabberAngles[1] && pawlServoAngles[1]==Constants.Climber.pawlClosed) {
      grabberAngles[0]=prevEncoderGrabberAngles[0];
    }


    if (isCalibrated){

      

      if ((grabberL0Encoder.getPosition()<gripperOpenMaximum+2 && grabberL0Encoder.getPosition()<grabberAngles[0]) ||
          (grabberL0Encoder.getPosition()>gripperClosedMinimum && grabberL0Encoder.getPosition()>grabberAngles[0]) ) {
        grabberL0PID.setReference(grabberAngles[0], ControlType.kPosition);
      }
      if ((grabberR0Encoder.getPosition()<gripperOpenMaximum+2 && grabberR0Encoder.getPosition()<grabberAngles[0]) ||
          (grabberR0Encoder.getPosition()>gripperClosedMinimum && grabberR0Encoder.getPosition()>grabberAngles[0])) {
        grabberR0PID.setReference(grabberAngles[0], ControlType.kPosition);
        SmartDashboard.putBoolean("moving R0", true);
      } else {
        SmartDashboard.putBoolean("moving R0", false);
      }
      if ((grabberL1Encoder.getPosition()<gripperOpenMaximum+2 && grabberL1Encoder.getPosition()<grabberAngles[1]) ||
          (grabberL1Encoder.getPosition()>gripperClosedMinimum && grabberL1Encoder.getPosition()>grabberAngles[1])) {
        grabberL1PID.setReference(grabberAngles[1], ControlType.kPosition);
      }
      if ((grabberR1Encoder.getPosition()<gripperOpenMaximum+2 && grabberR1Encoder.getPosition()<grabberAngles[1]) ||
          (grabberR1Encoder.getPosition()>gripperClosedMinimum && grabberR1Encoder.getPosition()>grabberAngles[1])) {
        grabberR1PID.setReference(grabberAngles[1], ControlType.kPosition);
      }


      if (grabberL0Encoder.getPosition()>gripperOpenMaximum+3 || grabberL0Encoder.getPosition()<gripperClosedMinimum-2){
        grabberL0.set(0);
      }
      if (grabberR0Encoder.getPosition()>gripperOpenMaximum+3 || grabberR0Encoder.getPosition()<gripperClosedMinimum-2){
        grabberR0.set(0);
      }
      if (grabberL1Encoder.getPosition()>gripperOpenMaximum+3 || grabberL1Encoder.getPosition()<gripperClosedMinimum-2){
        grabberL1.set(0);
      }
      if (grabberR1Encoder.getPosition()>gripperOpenMaximum+3 || grabberR1Encoder.getPosition()<gripperClosedMinimum-2){
        grabberR1.set(0);
      }

      

    } else if (isCalibrating){
      if (grabberR0.getOutputCurrent() > calibrationCurrent || grabberR0.getLastError() == REVLibError.kCANDisconnected) {
        grabberR0.set(0);
        grabberR0Encoder.setPosition(0);
        grabberAngles[0]=0;
        isGrabberCalibrated[0] = true;
      }
      if (grabberL0.getOutputCurrent() > calibrationCurrent || grabberL0.getLastError() == REVLibError.kCANDisconnected){
        grabberL0.set(0);
        grabberL0Encoder.setPosition(0);
        grabberAngles[0]=0;
        isGrabberCalibrated[1] = true;
      }
      if (grabberR1.getOutputCurrent() > calibrationCurrent || grabberR1.getLastError() == REVLibError.kCANDisconnected) {
        grabberR1.set(0);
        grabberR1Encoder.setPosition(0);
        grabberAngles[1]=0;
        isGrabberCalibrated[2] = true;
      }
      if (grabberL1.getOutputCurrent() > calibrationCurrent || grabberL1.getLastError() == REVLibError.kCANDisconnected){
        grabberL1.set(0);
        grabberL1Encoder.setPosition(0);
        grabberAngles[1]=0;
        isGrabberCalibrated[3] = true;
      }

      pawlServoAngles[0] = Constants.Climber.pawlOpen;
      pawlServoAngles[1] = Constants.Climber.pawlOpen;

      // :) set isCalibrated to true only if all in the list is set to true
      isCalibrated = isGrabberCalibrated[0] && isGrabberCalibrated[1] && isGrabberCalibrated[2] && isGrabberCalibrated[3];
      // :) also stop calibrating when the above conditions are met
      isCalibrating = !isCalibrated;

      prevEncoderGrabberAngles = encoderGrabberAngles;
      
    }
    

    // :) putting motors stuff on smartdashboard
    SmartDashboard.putNumber("Avg. Temp of grabber motors 0 (Celcius)", (grabberL0.getMotorTemperature()+grabberR0.getMotorTemperature())/2);
    SmartDashboard.putNumber("Avg. Temp of grabber motors 1 (Celcius)", (grabberL1.getMotorTemperature()+grabberR1.getMotorTemperature())/2);
    SmartDashboard.putNumber("Avg. Temp of climber motors (Celcius)", (climberMotorL.getMotorTemperature()+climberMotorR.getMotorTemperature())/2);
    SmartDashboard.putNumber("Grabber side 0", grabberAngles[0]);
    SmartDashboard.putNumber("Grabber side 1", grabberAngles[1]);
    SmartDashboard.putNumber("Grabber side 1 encoder", grabberR1Encoder.getPosition());
    SmartDashboard.putNumber("grabber r1 current", grabberR1.getOutputCurrent());
    SmartDashboard.putBooleanArray("isCalibratedArray", isGrabberCalibrated);
    SmartDashboard.putBoolean("isCalibrated", isCalibrated);
    SmartDashboard.putNumberArray("set grabber values", grabberAngles);
    SmartDashboard.putNumber("climber step", currentClimberStep);
    SmartDashboard.putBoolean("climb command", isRunningClimbCommand);
    SmartDashboard.putNumber("L spinner encoder", climberMotorLEncoder.getPosition());
    SmartDashboard.putNumber("theoretical climber angle", climberAngle);
    
    // SmartDashboard.putNumber("Current of climber L", climberMotorL.getOutputCurrent());
    // SmartDashboard.putNumber("Current of climber R", climberMotorR.getOutputCurrent());
    // SmartDashboard.putNumber("Shaktool angle", climberMotorLEncoder.getPosition());
    if (climberMotorR.getMotorTemperature()>maxTemp || climberMotorL.getMotorTemperature()>maxTemp ||
        (grabberL0.getMotorTemperature()>maxTemp && grabberL0.getMotorTemperature()<150) ||
        (grabberL1.getMotorTemperature()>maxTemp && grabberL1.getMotorTemperature()<150) ||
        (grabberR0.getMotorTemperature()>maxTemp && grabberR0.getMotorTemperature()<150) ||
        (grabberR1.getMotorTemperature()>maxTemp && grabberR1.getMotorTemperature()<150)) {
      isTooHot = true;
    } else {
      isTooHot = false;
    }
    prevStopped = isClimberStepStopped;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
