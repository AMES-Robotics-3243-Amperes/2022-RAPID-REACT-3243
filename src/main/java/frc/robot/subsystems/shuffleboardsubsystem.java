// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;



public class ShuffleboardSubsystem extends SubsystemBase {
  /** Creates a new shuffleboard. */

  ShuffleboardTab movementTab;
  ShuffleboardTab driverFeedbackTab;

  static SimpleWidget firstPowerShuffle;
  static SimpleWidget secondPowerShuffle;
  static SimpleWidget aCoeffShuffle;
  static SimpleWidget bCoeffShuffle;
  static SimpleWidget fastModeMultiplierShuffle;
  static SimpleWidget totalSpeedDamperShuffle;

  private static final AHRS imu = new AHRS();
  private static SimpleWidget roughxpos, roughypos, roughzpos;

  
  public ShuffleboardSubsystem() {
    driverFeedbackTab = Shuffleboard.getTab("Driverfeedback");
    movementTab = Shuffleboard.getTab("Drivetrain");
  // // I'm gonna cry


    firstPowerShuffle = movementTab.add("firstpower", Constants.Joysticks.firstPower);
    secondPowerShuffle = movementTab.add("secondpower", Constants.Joysticks.secondPower);
    aCoeffShuffle = movementTab.add("aCoeff", Constants.Joysticks.aCoeff);
    // ++ NOTE: we don't actually need to read anything for the b coefficient, because the b coefficient is just (1 - a coeff)
    bCoeffShuffle = movementTab.add("bCoeff", Constants.Joysticks.bCoeff);
    fastModeMultiplierShuffle = movementTab.add("fast mode mult", Constants.Joysticks.fastModeMaxMultiplier);
    totalSpeedDamperShuffle = movementTab.add("speed damper", Constants.Joysticks.driveSpeedDamper);



    roughxpos = driverFeedbackTab.add("roughx", imu.getDisplacementX()); 
    roughypos = driverFeedbackTab.add("roughy", imu.getDisplacementY());
    roughzpos = driverFeedbackTab.add("roughz", imu.getDisplacementZ());

  }

  public static double getFirstPower() {
    return (double)(firstPowerShuffle.getEntry().getNumber(Constants.Joysticks.firstPower));
    
  }

  public static double getSecondPower() {
    return (double)(secondPowerShuffle.getEntry().getNumber((double)Constants.Joysticks.secondPower));
    
  }
  public static double getaCoeff() {
    return (double)(aCoeffShuffle.getEntry().getDouble((double)Constants.Joysticks.aCoeff));
    
  }

  public static double getbCoeff(){
    // ++ this returns the b coefficient
    // ++ it's (1- a coeff) becasue that's what the
    return (double)(1.0 - getaCoeff());
  }

  public static double getFastmodeMultiplier(){
    return (double)(fastModeMultiplierShuffle.getEntry().getNumber((double)Constants.Joysticks.fastModeMaxMultiplier));
  }

  public static double getDriveSpeedDamper(){
    return (double)(totalSpeedDamperShuffle.getEntry().getDouble((double)Constants.Joysticks.driveSpeedDamper));
  }


  // thisIsCamelCase
  // ThisIsPascalCase
  // this_is_python_case_whatever_it's_really_called
  // thisiswhatjessedidwhenoriginallywritingthisclasscase

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
