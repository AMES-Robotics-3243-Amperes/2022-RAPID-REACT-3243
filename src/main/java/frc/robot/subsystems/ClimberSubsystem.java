// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {

  private CANSparkMax climber_motor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax grabber_1 = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax grabber_2 = new CANSparkMax(4, MotorType.kBrushless);

  public ClimberSubsystem () {
  }

  public void actuate_grabber(int grabber, double to_angle){
    // :) actuate grabber motors
  }

  public void spin_climber(double to_angle){
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
