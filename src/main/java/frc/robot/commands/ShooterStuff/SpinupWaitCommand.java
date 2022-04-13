// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterStuff;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/** ++ this command pauses in between the two balls shooting so the flywheel can spin back up */
public class SpinupWaitCommand extends CommandBase {


  private Timer clock = new Timer(); 
  public double pauseTime;

  /** Creates a new SpinupWaitCommand. */
  public SpinupWaitCommand(double waitTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    pauseTime = waitTime; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clock.reset();
    clock.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( clock.get() >= pauseTime);
  }
}
