// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;



public class shuffleboardsubsystem extends SubsystemBase {
  /** Creates a new shuffleboard. */

  ShuffleboardTab movementtab;
  ShuffleboardTab driverfeedbacktab;
  static SimpleWidget firstpowershuffle;
  static SimpleWidget secondpowershuffle;
  static SimpleWidget aCoeffshuffle;
  static SimpleWidget bCoeffShuffle;
  private static final AHRS imu = new AHRS();
  private static SimpleWidget roughxpos, roughypos, roughzpos;

  private static SendableChooser<Integer> m_FirstPower;


  public shuffleboardsubsystem() {
    driverfeedbacktab = Shuffleboard.getTab("Driverfeedback");
    movementtab = Shuffleboard.getTab("Drivetrain");
  // // I'm gonna cry
    firstpowershuffle = movementtab.add("firstpower", Constants.Joysticks.firstPower);
    secondpowershuffle = movementtab.add("secondpower", Constants.Joysticks.secondPower);
    aCoeffshuffle = movementtab.add("aCoeff", Constants.Joysticks.aCoeff);
    bCoeffShuffle = movementtab.add("bCoeff", Constants.Joysticks.bCoeff);
    roughxpos = driverfeedbacktab.add("roughx", imu.getDisplacementX());
    roughypos = driverfeedbacktab.add("roughy", imu.getDisplacementY());
    roughzpos = driverfeedbacktab.add("roughz", imu.getDisplacementZ());

    m_FirstPower = new SendableChooser<>();
    m_FirstPower.setDefaultOption("Good Power", 3);
    m_FirstPower.addOption("Bad Power", 4);

    movementtab.add(m_FirstPower);



  }

  public static int getfirstpower() {
    //return (int)(firstpowershuffle.getEntry().getDouble(Constants.Joysticks.firstPower));
    return m_FirstPower.getSelected();
    
  }

  public static int getsecondpower() {
    return (int)(secondpowershuffle.getEntry().getDouble(Constants.Joysticks.secondPower));
    
  }
  public static int getaCoeff() {
    return (int)(aCoeffshuffle.getEntry().getDouble(Constants.Joysticks.aCoeff));
    
  }

  public static int getbCoeff() {
    return (int)(bCoeffShuffle.getEntry().getDouble(Constants.Joysticks.bCoeff));
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
