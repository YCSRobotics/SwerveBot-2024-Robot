// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapFlipCmd extends Command {
  private final TrapArmSubsystem trapArmSubsystem;
  private final double trapPosition;

  public TrapFlipCmd (TrapArmSubsystem trapArmSubsystem, double trapPosition) {
    this.trapArmSubsystem = trapArmSubsystem;
    this.trapPosition = trapPosition;
  }

 
  @Override
  public void initialize() {
    trapArmSubsystem.flip(trapPosition);
  }

  
  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
