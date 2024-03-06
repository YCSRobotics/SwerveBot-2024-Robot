package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class HangerSubsystem extends SubsystemBase {
    public CANSparkMax m_hangerMotor;
    private final SparkPIDController m_hangerPIDController;
    // private final RelativeEncoder m_encoder;

    // PID Coefficient
    //private double kP, kI, kD;

    public HangerSubsystem(int motorID) {
        m_hangerMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        m_hangerMotor.setSmartCurrentLimit(Constants.Mechanisms.hangerCurrentLimit);
        
        m_hangerPIDController = m_hangerMotor.getPIDController();

        // Set the PID coefficients
        // kP = 0.05;
        // kI = 0;
        // kD = 0;

        // Set PID coefficients
        m_hangerPIDController.setP(Constants.Mechanisms.hangerPIDControllerkP);
        m_hangerPIDController.setI(Constants.Mechanisms.hangerPIDControllerkI);
        m_hangerPIDController.setD(Constants.Mechanisms.hangerPIDControllerkD);
    }

    public void setHangerVelocityTarget(double targetVelocity) {
        m_hangerPIDController.setReference(targetVelocity, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift Current" + m_hangerMotor.getDeviceId(), m_hangerMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Lift Velocity", m_encoder.getVelocity());
    }

    public void stopMotor() {
        m_hangerMotor.set(0);
    }
}
