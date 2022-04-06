// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterStuff.AutoShooterRoutineCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.JoyUtil;
import frc.robot.subsystems.LimelightSubsystem;


/** ++ this class rumbles the controller in a specific pattern if the shoot routine fails */
public class ShootRoutineFailedRumble extends CommandBase {

  /* ++ the rumble pattern should be:
  * on for 0.2 seconds
  * off for 0.3 seconds
  * on for 0.2 seconds
  * turn off
  */





  private final JoyUtil controller;
  private final Timer clock = new Timer();

  boolean inFirstRumble;
  boolean inSecondRumble;


  /** Creates a new RoutineFailedRumble. */
  public ShootRoutineFailedRumble(JoyUtil secondaryController) {

    controller = secondaryController;


    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clock.reset();
    clock.start();

    controller.stopRumbleRight(); // :) this is code

    inFirstRumble = true;
    inSecondRumble = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.rumbleRight(1.0);

    if ( clock.get() >= 0.2 ) {
      inFirstRumble = false;
    }
    if ( (clock.get() >= 0.5) && (clock.get() <= 0.7) ) {
      inSecondRumble = true;
    }

    if (inFirstRumble || inSecondRumble) {
      controller.rumbleRight(1.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.stopRumbleRight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( (clock.get() >= 0.7) || LimelightSubsystem.continueShooterRoutine);
  }
}
