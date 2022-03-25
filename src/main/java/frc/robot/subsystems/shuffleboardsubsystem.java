// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// ++ comment for commit test

package frc.robot.subsystems;

import frc.robot.subsystems.IMUSubsystem;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;



public class ShuffleboardSubsystem extends SubsystemBase {
  /** Creates a new shuffleboard. */

  // ++ creates IMUSubsystem object 
  IMUSubsystem IMUSubsystem = new IMUSubsystem();

  // ++ declare tabs
  ShuffleboardTab movementTab;
  ShuffleboardTab driverFeedbackTab;
  ShuffleboardTab IMUTab;
  ShuffleboardTab shooterTab;

  // ++ declare widgets
    // ++ drive stuff
  static SimpleWidget firstPowerShuffle;
  static SimpleWidget secondPowerShuffle;
  static SimpleWidget aCoeffShuffle;
  static SimpleWidget bCoeffShuffle;
  static SimpleWidget fastModeMultiplierShuffle;
  static SimpleWidget totalSpeedDamperShuffle;
    // ++ driver feedback



  // ++ this gives a selector thing in Shuffleboard that lets you switch the first power
  private static SendableChooser<Integer> m_FirstPower;


  private static SimpleWidget roughxpos, roughypos, roughzpos;
  
  public ShuffleboardSubsystem() {
    // ++ define all the tabs
    driverFeedbackTab = Shuffleboard.getTab("Driverfeedback");
    movementTab = Shuffleboard.getTab("Drivetrain");
    IMUTab = Shuffleboard.getTab("IMU");
    shooterTab = Shuffleboard.getTab("shooter");

    



  // // I'm gonna cry
  // ++ me too I'm freezing rn cottonwood's area is coldddddd



    // ++ ===================== DRIVETRAIN WIDGETS/STUFF ===============================================
    firstPowerShuffle = movementTab.add("firstpower", Constants.Joysticks.firstPower);
    secondPowerShuffle = movementTab.add("secondpower", Constants.Joysticks.secondPower);
    aCoeffShuffle = movementTab.add("aCoeff", Constants.Joysticks.aCoeff);
    // ++ NOTE: we don't actually need to read anything for the b coefficient, because the b coefficient is just (1 - a coeff)
    bCoeffShuffle = movementTab.add("bCoeff", Constants.Joysticks.bCoeff);
    fastModeMultiplierShuffle = movementTab.add("fast mode mult", Constants.Joysticks.fastModeMaxMultiplier);
    totalSpeedDamperShuffle = movementTab.add("speed damper", Constants.Joysticks.driveSpeedDamper);

    m_FirstPower = new SendableChooser<>();
    m_FirstPower.setDefaultOption("Good Power", 3);
    m_FirstPower.addOption("Bad Power", 4);

    movementTab.add(m_FirstPower);
    // ++  =================== END DRIVETRAIN WIDGETS/STUFF ==============================================


  }

  // ++ =========================================== DRIVETRAIN METHODS ===============================================
  public static int getFirstPower() {
    //return (int)(firstpowershuffle.getEntry().getDouble(Constants.Joysticks.firstPower));
    return m_FirstPower.getSelected();
    
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
  // ++ ============================= END DRIVETRAIN METHODS ==================================================


  // ++ ================= IMU stuff ===============================

  // roughxpos = driverFeedbackTab.add("roughx", IMUSubsystem.getXPosition()); 
  // roughypos = driverFeedbackTab.add("roughy", IMUSubsystem.getYPosition());
  // roughzpos = driverFeedbackTab.add("roughz", IMUSubsystem.getZPosition());

  // ++ ================= end IMU stuff ===========================



  /* ++
    thisIsCamelCase
    ThisIsPascalCase
   this_is_python_case_whatever_it's_really_called
   thisiswhatjessedidwhenoriginallywritingthisclasscase
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    firstPowerShuffle.getEntry().setDouble(m_FirstPower.getSelected());
    bCoeffShuffle.getEntry().setDouble(1.0 - aCoeffShuffle.getEntry().getDouble(Constants.Joysticks.aCoeff));
  }
}
