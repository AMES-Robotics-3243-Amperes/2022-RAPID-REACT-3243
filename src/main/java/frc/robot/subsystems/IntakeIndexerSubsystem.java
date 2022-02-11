// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class IntakeIndexerSubsystem extends SubsystemBase {
  private final CANSparkMax dropMotor;
  private final CANSparkMax intakeMotor;
  private final CANSparkMax indexMotor;
  private final RelativeEncoder dropEncoder;
  /** Creates a new IntakeIndexerSubsystem. */
  public IntakeIndexerSubsystem() {
    dropMotor = new CANSparkMax(Constants.IntakeIndexer.dropMotorID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.IntakeIndexer.intakeMotorID, MotorType.kBrushless);
    indexMotor = new CANSparkMax(Constants.IntakeIndexer.indexMotorID, MotorType.kBrushless);

    dropEncoder = dropMotor.getEncoder();
    dropEncoder.setPosition(0);
  }

  public void setDropPos(double pos) {
    SparkMaxPIDController pid = dropMotor.getPIDController();
    pid.setReference(pos, ControlType.kPosition);
  }

  public double getDropPos() {
    return dropEncoder.getPosition();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIndexerSpeed(double speed) {
    indexMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// ++ Coment to make sure I didn't break everything.