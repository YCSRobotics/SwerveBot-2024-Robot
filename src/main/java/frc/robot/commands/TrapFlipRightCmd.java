// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TrapArmSubsystem;

public class TrapFlipRightCmd extends Command {
  private final TrapArmSubsystem trapArmSubsystem;
  private final double trapPosition;

  public TrapFlipRightCmd (TrapArmSubsystem trapArmSubsystem, double trapPosition) {
    this.trapArmSubsystem = trapArmSubsystem;
    this.trapPosition = trapPosition;

    addRequirements(trapArmSubsystem);
  }

 
  @Override
  public void initialize() {
    trapArmSubsystem.flip(Constants.Mechanisms.trapArmFlipRightSpeed);
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
