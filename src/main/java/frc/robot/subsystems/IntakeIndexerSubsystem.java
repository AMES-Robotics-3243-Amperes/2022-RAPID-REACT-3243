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
  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder indexEncoder;

  private final SparkMaxPIDController dropPID;
  private final SparkMaxPIDController intakePID;
  private final SparkMaxPIDController indexPID;

  /** Creates a new IntakeIndexerSubsystem. */
  public IntakeIndexerSubsystem() {
    dropMotor = new CANSparkMax(Constants.IntakeIndexer.dropMotorID, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Constants.IntakeIndexer.intakeMotorID, MotorType.kBrushless);
    indexMotor = new CANSparkMax(Constants.IntakeIndexer.indexMotorID, MotorType.kBrushless);

    dropEncoder = dropMotor.getEncoder();
    intakeEncoder = intakeMotor.getEncoder();
    indexEncoder = indexMotor.getEncoder();

    dropPID = dropMotor.getPIDController();
    intakePID = intakeMotor.getPIDController();
    indexPID = indexMotor.getPIDController();

    // ~~ Conversion ratios to account for gearbox ratios
    dropEncoder.setVelocityConversionFactor(Constants.IntakeIndexer.dropVelocityConversionRatio);
    dropEncoder.setPositionConversionFactor(Constants.IntakeIndexer.dropPositionConversionRatio);
    intakeEncoder.setVelocityConversionFactor(Constants.IntakeIndexer.intakeVelocityConversionRatio);
    intakeEncoder.setPositionConversionFactor(Constants.IntakeIndexer.intakePositionConversionRatio);
    indexEncoder.setVelocityConversionFactor(Constants.IntakeIndexer.indexVelocityConversionRatio);
    indexEncoder.setPositionConversionFactor(Constants.IntakeIndexer.indexPositionConversionRatio);

    dropEncoder.setPosition(0);
    intakeEncoder.setPosition(0);
    indexEncoder.setPosition(0);
  }

  public void setDropSpeed(double speed) {
    dropMotor.set(speed);
  }

  public void setDropPos(double pos) {
    // ~~ This method uses the CANSparkMax built-in PID controller to set a target position
    dropPID.setReference(pos, ControlType.kPosition);
  }

  public double getDropPos() {
    // ~~ This method returns the position of the drop bar
    return dropEncoder.getPosition();
  }

  public void setIntakeSpeed(double speed) {
    // ~~ This method spins the intake bars
    intakeMotor.set(speed);
  }

  public void setIndexerSpeed(double speed) {
    // ~~ This method spins the indexer wheels.
    indexMotor.set(speed);
  }

  public void stepIndexer(double rotations) {
    // ~~ Moves the indexer wheels a set amount
    indexEncoder.setPosition(0);
    indexPID.setReference(rotations, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// ~~ Coment to make sure I didn't break everything.