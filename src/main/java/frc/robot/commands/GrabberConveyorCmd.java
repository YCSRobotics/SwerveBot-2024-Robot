// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.ColorSensorV3;

public class GrabberConveyorCmd extends Command {
  private final GrabberSubsystem grabberSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private final XboxController controller;


  private static final double grabberTargetSpeed = 0.5;
  private static final double conveyorTargetSpeed = 0.5;


  /** Creates a new GrabberConveyorCmd. */
  public GrabberConveyorCmd(GrabberSubsystem grabberSubsystem, ConveyorSubsystem conveyorSubsystem) {
    this.grabberSubsystem = grabberSubsystem;
    this.conveyorSubsystem = conveyorSubsystem;
    this.controller = controller;
      
    addRequirements(grabberSubsystem, conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controller.getRawButtonPressed(1) && !isFieldElementInPosition()) {
      grabberSubsystem.setGrabberTargetSpeed(grabberTargetSpeed);
      conveyorSubsystem.setConveyorTargetSpeed(conveyorTargetSpeed);

    } else {
      grabberSubsystem.setGrabberTargetSpeed(0);
      conveyorSubsystem.setConveyorTargetSpeed(0);
    }
  }

  private boolean isFieldElementInPosition() {
    return proximitySensor.getP > 100;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
