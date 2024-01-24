// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherSubsystem;

public class LauncherCmd extends Command {
  private final LauncherSubsystem launcherSubsystem;
  private final double launcherTargetSpeed;
  

  /** Creates a new SetlauncherSpeedCmd. */
  public LauncherCmd(LauncherSubsystem launcherSubsystem, double launcherTargetSpeed) {
    this.launcherSubsystem = launcherSubsystem;
    this.launcherTargetSpeed = launcherTargetSpeed;
    
    addRequirements(launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherSubsystem.setLauncherTargetSpeed(launcherTargetSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.setLauncherTargetSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
