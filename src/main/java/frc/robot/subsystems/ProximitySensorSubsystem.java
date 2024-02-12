// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ProximitySensorSubsystem extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 proximitySensor = new ColorSensorV3(i2cPort);

  /** Creates a new ProximitySensorSubsystem. */
  public ProximitySensorSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isFieldElementInPosition() {
    return proximitySensor.getProximity() > (1500);
  }
}
