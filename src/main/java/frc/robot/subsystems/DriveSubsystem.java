// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {

  private MecanumDrive speeds;
  private CANSparkMax frontleftmotor = new CANSparkMax( Constants.Motors.FrontLeft, MotorType.kBrushless);
  private CANSparkMax frontrightmotor = new CANSparkMax( Constants.Motors.FrontRight, MotorType.kBrushless);
  private CANSparkMax backleftmotor = new CANSparkMax( Constants.Motors.BackLeft, MotorType.kBrushless);
  private CANSparkMax backrightmotor = new CANSparkMax( Constants.Motors.BackRight, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    frontrightmotor.setInverted(true);
    backrightmotor.setInverted(true);
    speeds = new MecanumDrive(frontleftmotor, backleftmotor, frontrightmotor, backrightmotor);
  }

  public void setMotors (double X_speed, double Y_speed, double Z_rotation) {
    speeds.driveCartesian(Y_speed, X_speed, Z_rotation);
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
