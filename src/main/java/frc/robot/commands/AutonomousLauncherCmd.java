// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LauncherSubsystem;

public class AutonomousLauncherCmd extends Command {

  private final LauncherSubsystem launcherSubsystem;
  private final double launcherTargetSpeed;
  private final durationSeconds;

  /** Creates a new AutonomousCmd. */
  public AutonomousLauncherCmd(LauncherSubsystem launcherSubsystem, double launcherTargetSpeed, double durationSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSubsystem = launcherSubsystem;
    this.launcherTargetSpeed = launcherTargetSpeed;
    this.durationSeconds = durationSeconds;
    addRequirements(launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launcherSubsystem.setLauncherTargetSpeed(launcherTargetSpeed);

    schedule(new WaitCommand(durationSeconds));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSubsystem.setLauncherTargetSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}