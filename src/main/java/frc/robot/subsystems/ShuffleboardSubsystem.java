// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// ++ comment for commit test

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.ControlOption;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.function.BooleanSupplier;


public class ShuffleboardSubsystem extends SubsystemBase {
  /** Creates a new shuffleboard. */

  // ++ declare tabs
  ShuffleboardTab movementTab, driverFeedbackTab, IMUTab, shooterTab;

  // ++ declare widgets
    // ++ Drive Control Stuff
  static SimpleWidget firstPowerShuffle, secondPowerShuffle, aCoeffShuffle, bCoeffShuffle, fastModeMultiplierShuffle, totalSpeedDamperShuffle;
    // ~~ IMU Stuff
  static SimpleWidget yawShuffle, pitchShuffle, rollShuffle, xVelocityShuffle, yVelocityShuffle, zVelocityShuffle, xPositionShuffle, yPositionShuffle, zPositionShuffle;
  // ££ Driver Feedback
  static ComplexWidget field;
  // ££ Shooter Feedback
  static SimpleWidget seesTarget;
    // ++ shooter stuff
  static SimpleWidget 
      flywheelSpeed, 
      
      flywheelPGain, 
      flywheelIGain, 
      flywheelDGain, 
      flywheelFFGain, 
      servoTarget, 
      servoLastSet,
      routineConting, 
      hubDistance;

  // ++ this gives a selector thing in Shuffleboard that lets you switch the first power
  private static SendableChooser<ControlOption> m_JoyCurve;

  private static SendableChooser<Pose2d> m_StartingPosition, m_Target;


  private static SimpleWidget roughxpos, roughypos, roughzpos;
  
  public ShuffleboardSubsystem() {
    // ++ define all the tabs
      driverFeedbackTab = Shuffleboard.getTab("Driverfeedback");
      movementTab = Shuffleboard.getTab("Drivetrain");
      IMUTab = Shuffleboard.getTab("IMU");
      shooterTab = Shuffleboard.getTab("Shooter");

    



  // // I'm gonna cry\
  // ++ me too I'm freezing rn cottonwood's area is coldddddd
  // :) loser my garage is colder


    // ++ ==================== DRIVETRAIN WIDGETS/STUFF ===============================================
    firstPowerShuffle = movementTab.add("firstpower", Constants.Joysticks.firstPower);
    secondPowerShuffle = movementTab.add("secondpower", Constants.Joysticks.secondPower);
    aCoeffShuffle = movementTab.add("aCoeff", Constants.Joysticks.aCoeff);
    // ++ NOTE: we don't actually need to read anything for the b coefficient, because the b coefficient is just (1 - a coeff)
    bCoeffShuffle = movementTab.add("bCoeff", Constants.Joysticks.bCoeff);
    fastModeMultiplierShuffle = movementTab.add("fast mode mult", Constants.Joysticks.fastModeMaxMultiplier);
    totalSpeedDamperShuffle = movementTab.add("speed damper", Constants.Joysticks.driveSpeedDamper);

    m_JoyCurve = new SendableChooser<>();
    m_JoyCurve.setDefaultOption("Good Option", new ControlOption(3, 1, 0.7));
    m_JoyCurve.addOption("Bad Option", new ControlOption(4, 1, 1));

    m_StartingPosition = new SendableChooser<>();
    m_StartingPosition.setDefaultOption("Sector 1 Left", new Pose2d(6.1363, 5.9592, new Rotation2d(2.35619449)));
    m_StartingPosition.addOption("Sector 1 Right", new Pose2d(5.7595, 5.5824, new Rotation2d(2.35619449)));
    m_StartingPosition.addOption("Sector 2 Left", new Pose2d(5.4452, 4.8236, new Rotation2d(-3.141592654)));
    m_StartingPosition.addOption("Sector 2 Right", new Pose2d(5.4452, 4.2909, new Rotation2d(-3.141592654)));
    m_StartingPosition.addOption("Sector 3 Left", new Pose2d(6.3852, 2.0215, new Rotation2d(-2.35619449)));
    m_StartingPosition.addOption("Sector 3 Right", new Pose2d(6.7620, 1.6447, new Rotation2d(-2.35619449)));
    m_StartingPosition.addOption("Sector 4 Left", new Pose2d(7.5208, 1.3304, new Rotation2d(-1.570796327)));
    m_StartingPosition.addOption("Sector 4 Right", new Pose2d(8.0535, 1.3304, new Rotation2d(-1.570796327)));

    m_Target = new SendableChooser<>();
    m_Target.setDefaultOption("Left Ball", new Pose2d(4.9983, 6.2739, new Rotation2d()));
    m_Target.addOption("Center Ball", new Pose2d(4.9983, 1.9557, new Rotation2d()));
    m_Target.addOption("Right Ball", new Pose2d(7.4714, 0.3033, new Rotation2d()));


    movementTab.add(m_JoyCurve);
    movementTab.add(m_StartingPosition);
    movementTab.add(m_Target);
    // ++  =================== END DRIVETRAIN WIDGETS/STUFF ==============================================



    // ~~ ==================== IMU WIDGETS/STUFF =========================================================
    yawShuffle = IMUTab.add("Yaw", 0);
    pitchShuffle = IMUTab.add("Pitch", 0);
    rollShuffle = IMUTab.add("Roll", 0);
    xVelocityShuffle = IMUTab.add("X Velocity", 0);
    yVelocityShuffle = IMUTab.add("Y Velocity", 0);
    zVelocityShuffle = IMUTab.add("Z Velocity", 0);
    xPositionShuffle = IMUTab.add("X Position", 0);
    yPositionShuffle = IMUTab.add("Y Position", 0);
    zPositionShuffle = IMUTab.add("Z Position", 0);
    // ~~ ==================== END IMU WIDGETS/STUFF =====================================================
    
      // ££ ============================= DRIVER FEEDBACK ================================================
      field = driverFeedbackTab.add(DriveSubsystem.getField());
      // ££ ==================== SHOOTER FEEDBACK ==========================================================
      seesTarget = driverFeedbackTab.add("Sees Target is True", LimelightSubsystem.isTargetValid());
      



    // ++ ==================== SHOOTER STUFF =============================================
    flywheelSpeed = shooterTab.add("flywheel speed", 0.0);

    flywheelPGain = shooterTab.add("flywheel P gain", Constants.Shooter.flywheelPGain);
    flywheelIGain = shooterTab.add("flywheel I gain", Constants.Shooter.flywheelIGain);
    flywheelDGain = shooterTab.add("flywheel D gain", Constants.Shooter.flywheelDGain);
    flywheelFFGain = shooterTab.add("flywheel FF gain", Constants.Shooter.flywheelFFGain);

    servoTarget = shooterTab.add("servo target", 0.0);

    servoLastSet = shooterTab.add("last servo set", 0.0);

    // hubDistance = shooterTab.add("distance from hub", 999.0);

    // routineConti
    // ++ ==================== END SHOOTER STUFF =========================================

  }

  // ++ =========================================== DRIVETRAIN METHODS ===================================
  public static int getFirstPower() {
    //return (int)(firstpowershuffle.getEntry().getDouble(Constants.Joysticks.firstPower));
    return m_JoyCurve.getSelected().getFirstPower();
    
  }

  public static int getSecondPower() {
    // return (double)(secondPowerShuffle.getEntry().getNumber((double)Constants.Joysticks.secondPower));
    return m_JoyCurve.getSelected().getSecondPower();
    
  }
  public static double getaCoeff() {
    // return (double)(aCoeffShuffle.getEntry().getDouble((double)Constants.Joysticks.aCoeff));
    return m_JoyCurve.getSelected().getACoefficient();
    
  }

  public static double getbCoeff(){
    // ++ this returns the b coefficient
    // ++ it's (1- a coeff) becasue that's what the
    //return (double)(1.0 - getaCoeff());
    return m_JoyCurve.getSelected().getBCoefficient();
  }

  public static double getFastmodeMultiplier(){
    return (double)(fastModeMultiplierShuffle.getEntry().getNumber((double)Constants.Joysticks.fastModeMaxMultiplier));
  }

  public static double getDriveSpeedDamper(){
    return (double)(totalSpeedDamperShuffle.getEntry().getDouble((double)Constants.Joysticks.driveSpeedDamper));
  }

  public static Pose2d getStartingPose() {
    return m_StartingPosition.getSelected();
  }

  public static Pose2d getTargetPose() {
    return m_Target.getSelected();
  }
  // ++ ============================= END DRIVETRAIN METHODS =============================================


  // ++ ============================= SHOOTER METHODS ===================================================
  /** ++ this method returns the pid values for the shooter as an array,
   * in the order P, I, D, FF */
  public static double[] getFlywheelPIDValues(){
    double PIDFValues[] = {
    flywheelPGain.getEntry().getDouble( Constants.Shooter.flywheelPGain),
    flywheelIGain.getEntry().getDouble( Constants.Shooter.flywheelIGain),
    flywheelDGain.getEntry().getDouble( Constants.Shooter.flywheelDGain),
    flywheelFFGain.getEntry().getDouble( Constants.Shooter.flywheelFFGain),
    };

    return PIDFValues;
    
  }

    // ++ --- display methods

  // public static void displayFlywheelSpeed(double speed) {
  //   flywheelSpeed.getEntry().setDouble( speed );
  // }

  public static void displayCalcDistance(double distance) {
    hubDistance.getEntry().setDouble( distance );
  }

  public static void displayPrevServoSet(double lastSet) {
    servoLastSet.getEntry().setDouble(lastSet);
  }

    // ++ --- read methods

  public static double readTargetHoodAngle() {
    return servoTarget.getEntry().getDouble(-1);
  }

  public static double readTargetFlywheelRPM(){
    return flywheelSpeed.getEntry().getDouble(-1);
  }


  // ++ ============================= END SHOOTER METHODS ===============================================

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
   Camel Case is superior
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // ~~ ==================== JOYSTICK CURVE OUTPUTS ====================================================
    firstPowerShuffle.getEntry().setDouble(getFirstPower());
    secondPowerShuffle.getEntry().setDouble(getSecondPower());
    aCoeffShuffle.getEntry().setDouble(getaCoeff());
    bCoeffShuffle.getEntry().setDouble(getbCoeff());
    // ~~ ==================== END JOYSTICK CURVE OUTPUTS ================================================

    // ~~ ==================== IMU OUTPUTS ===============================================================
    if (DriverStation.isTest()) {
    yawShuffle.getEntry().setDouble(IMUSubsystem.getYaw());
    pitchShuffle.getEntry().setDouble(IMUSubsystem.getPitch());
    rollShuffle.getEntry().setDouble(IMUSubsystem.getRoll());
    xVelocityShuffle.getEntry().setDouble(IMUSubsystem.getXVelocity());
    yVelocityShuffle.getEntry().setDouble(IMUSubsystem.getYVelocity());
    zVelocityShuffle.getEntry().setDouble(IMUSubsystem.getZVelocity());
    xPositionShuffle.getEntry().setDouble(IMUSubsystem.getXPosition());
    yPositionShuffle.getEntry().setDouble(IMUSubsystem.getYPosition());
    zPositionShuffle.getEntry().setDouble(IMUSubsystem.getZPosition());
    // ~~ ==================== END IMU OUTPUTS ===========================================================

  }
}
}
