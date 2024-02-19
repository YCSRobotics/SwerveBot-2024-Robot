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
    public CANSparkMax m_leftHangerMotor;
    public CANSparkMax m_rightHangerMotor;
    private final SparkPIDController m_leftHangerPIDController;
    private final SparkPIDController m_rightHangerPIDController;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;
    private final Pigeon2 gyro;

    public HangerSubsystem() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        System.out.println("Start controller config");
        m_leftHangerMotor = new CANSparkMax(Constants.Mechanisms.leftHangerMotorID, MotorType.kBrushless);
        m_rightHangerMotor = new CANSparkMax(Constants.Mechanisms.rightHangerMotorID, MotorType.kBrushless);
        m_leftHangerMotor.setSmartCurrentLimit(Constants.Mechanisms.hangerCurrentLimit);
        m_rightHangerMotor.setSmartCurrentLimit(Constants.Mechanisms.hangerCurrentLimit);
        m_leftEncoder = m_leftHangerMotor.getEncoder();
        m_rightEncoder = m_rightHangerMotor.getEncoder();
        
        m_leftHangerPIDController = m_leftHangerMotor.getPIDController();
        m_rightHangerPIDController = m_rightHangerMotor.getPIDController();
        m_leftHangerPIDController.setP(Constants.Mechanisms.hangerPIDControllerkP);
        m_leftHangerPIDController.setI(Constants.Mechanisms.hangerPIDControllerkI);
        m_leftHangerPIDController.setD(Constants.Mechanisms.hangerPIDControllerkD);
        m_rightHangerPIDController.setP(Constants.Mechanisms.hangerPIDControllerkP);
        m_rightHangerPIDController.setI(Constants.Mechanisms.hangerPIDControllerkI);
        m_rightHangerPIDController.setD(Constants.Mechanisms.hangerPIDControllerkD);
    }

    public void setHangerPositionTarget(double targetPosition) {
        m_leftHangerMotor.getPIDController().setReference(targetPosition, ControlType.kPosition);
        m_rightHangerMotor.getPIDController().setReference(targetPosition, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Motor Current", m_leftHangerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Motor Current", m_rightHangerMotor.getOutputCurrent());
        SmartDashboard.putNumber("Left Lift Encoder", m_leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Lift Encoder", m_rightEncoder.getPosition());
    }
}
