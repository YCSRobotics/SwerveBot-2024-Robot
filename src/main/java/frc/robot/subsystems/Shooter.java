// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private CANSparkMax m_flywheelMotorLeft;
    private CANSparkMax m_flywheelMotorRight;
    // private CANSparkMax m_flywheelMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    // private CANSparkMax m_flywheelMotorRight = new CANSparkMax(0, MotorType.kBrushless);
    // private SparkPIDController m_flywheelMotorLeftPidController;
    // private SparkPIDController m_flywheelMotorRightPidController;
}

/**public class Shooter extends SubsystemBase {

  // Creates a new Shooter.
  public Shooter() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}*/