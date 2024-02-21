package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class HangerSubsystem extends SubsystemBase {
    public CANSparkMax m_hangerMotor;
    private final SparkPIDController m_hangerPIDController;
    private final RelativeEncoder m_encoder;

    public HangerSubsystem(int HangerMotorID, double targetVelocity) {
        m_hangerMotor = new CANSparkMax(HangerMotorID, MotorType.kBrushless);
        m_hangerMotor.setSmartCurrentLimit(Constants.Mechanisms.hangerCurrentLimit);
        m_encoder = m_hangerMotor.getEncoder();
        
        m_hangerPIDController = m_hangerMotor.getPIDController();
        m_hangerPIDController.setP(Constants.Mechanisms.hangerPIDControllerkP);
        m_hangerPIDController.setI(Constants.Mechanisms.hangerPIDControllerkI);
        m_hangerPIDController.setD(Constants.Mechanisms.hangerPIDControllerkD);
    }

    public void setHangerVelocityTarget(double targetVelocity) {
        m_hangerMotor.getPIDController().setReference(targetVelocity, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift Current", m_hangerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Lift Velocity", m_encoder.getVelocity());
    }
}
