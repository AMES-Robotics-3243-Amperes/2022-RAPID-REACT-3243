// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class shuffleboardsubsystem extends SubsystemBase {
  /** Creates a new shuffleboard. */
  public shuffleboardsubsystem() {
    
  //I'm gonna cry
    ShuffleboardTab movementtab = Shuffleboard.getTab("Drivetrain");
    SimpleWidget firstpowershuffle;
    SimpleWidget secondpowershuffle;
    SimpleWidget aCoeffshuffle;
    SimpleWidget bCoeffShuffle;
    firstpowershuffle = movementtab.add("firstpower", Constants.Joysticks.firstPower);
    secondpowershuffle = movementtab.add("secondpower", Constants.Joysticks.secondPower);
    aCoeffshuffle = movementtab.add("aCoeff", Constants.Joysticks.aCoeff);
    bCoeffShuffle = movementtab.add("bCoeff", Constants.Joysticks.bCoeff);
    
    
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
