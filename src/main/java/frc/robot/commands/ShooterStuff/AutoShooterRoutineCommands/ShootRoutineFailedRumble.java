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

  double onTime = 0.1;
  double pauseTime = 0.5;

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

    if (controller != null){
      controller.stopRumbleRight(); // :) this is code
    }

    inFirstRumble = false;
    inSecondRumble = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controller.rumbleRight(1.0);

    if ( clock.get() >= 0.0 ) {
      inFirstRumble = true;
    }
    if ( clock.get() >= onTime ) {
      inFirstRumble = false;
    }
    if ( (clock.get() >= pauseTime) && (clock.get() >= (onTime + pauseTime)) ) {
      inSecondRumble = true;
    } 

    if (controller != null){
      if (inFirstRumble || inSecondRumble) {
        controller.rumbleRight(1.0);
      } else {
        controller.stopRumbleRight();
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (controller != null){
      controller.stopRumbleRight();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( (clock.get() >= (onTime + pauseTime)) || LimelightSubsystem.continueShooterRoutine);
  }
}
