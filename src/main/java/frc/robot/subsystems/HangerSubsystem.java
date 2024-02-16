package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangerSubsystem extends SubsystemBase {
    public CANSparkMax m_leftArmMotor;
    public CANSparkMax m_rightArmMotor;
    private final PIDController m_leftArmPIDController;
    private final PIDController m_rightArmPIDController;
    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    public HangerSubsystem() {
        m_leftArmMotor = new CANSparkMax(Constants.Mechanisms.leftArmMotorID, MotorType.kBrushless);
        m_rightArmMotor = new CANSparkMax(Constants.Mechanisms.rightArmMotorID, MotorType.kBrushless);
        m_leftArmMotor.setSmartCurrentLimit(Constants.Mechanisms.hangerCurrentLimit);
        m_rightArmMotor.setSmartCurrentLimit(Constants.Mechanisms.hangerCurrentLimit);
        m_leftEncoder = m_leftArmMotor.getEncoder();
        m_rightEncoder = m_rightArmMotor.getEncoder();

        // m_leftArmPIDController = new PIDController(Constants.Mechanisms.hangerPIDControllerkP, 
        //                                            Constants.Mechanisms.hangerPIDControllerkI, 
        //                                            Constants.Mechanisms.hangerPIDControllerkD);

        m_leftArmPIDController = m_leftArmMotor.getPIDController();

        m_rightArmPIDController = new PIDController(Constants.Mechanisms.hangerPIDControllerkP, 
                                                    Constants.Mechanisms.hangerPIDControllerkI, 
                                                    Constants.Mechanisms.hangerPIDControllerkD);
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        m_leftArmPIDController.setPID(kP, kI, kD);
        m_rightArmPIDController.setPID(kP, kI, kD);
    }

    public void setHangerPositionTarget(double targetPosition) {
        m_leftArmMotor.getPIDController().setReference(targetPosition, ControlType.kPosition);
        m_rightArmMotor.getPIDController().setReference(targetPosition, ControlType.kPosition);
    }

    @Override
    public void periodic() {

    }
}
