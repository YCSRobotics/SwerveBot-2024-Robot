package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase{
    public CANSparkMax m_leftLauncherMotor;
    public CANSparkMax m_rightLauncherMotor;
    private final SparkPIDController m_leftlauncherPIDController;
    private final SparkPIDController m_rightLauncherPIDController;

    public LauncherSubsystem() {
        m_leftLauncherMotor = new CANSparkMax(Constants.Mechanisms.leftLauncherMotorID, MotorType.kBrushless);
        m_rightLauncherMotor = new CANSparkMax(Constants.Mechanisms.rightLauncherMotorID, MotorType.kBrushless);
        m_leftLauncherMotor.setSmartCurrentLimit(Constants.Mechanisms.launcherCurrentLimit);
        m_rightLauncherMotor.setSmartCurrentLimit(Constants.Mechanisms.launcherCurrentLimit);

        m_leftlauncherPIDController = m_leftLauncherMotor.getPIDController();

        m_leftlauncherPIDController.setP(Constants.Mechanisms.launcherPIDControllerkP);
        m_leftlauncherPIDController.setP(Constants.Mechanisms.launcherPIDControllerkI);
        m_leftlauncherPIDController.setP(Constants.Mechanisms.launcherPIDControllerkD);

        m_rightLauncherPIDController = m_rightLauncherMotor.getPIDController();

        m_rightLauncherPIDController.setP(Constants.Mechanisms.launcherPIDControllerkP);
        m_rightLauncherPIDController.setP(Constants.Mechanisms.launcherPIDControllerkI);
        m_rightLauncherPIDController.setP(Constants.Mechanisms.launcherPIDControllerkD);
    }

    public void setLauncherVelocityTarget(double targetVelocity) {
        m_leftlauncherPIDController.setReference(targetVelocity, ControlType.kVelocity);
        m_leftlauncherPIDController.setReference(targetVelocity, ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        
    }

    public void stopMotor() {
        m_leftLauncherMotor.set(0);
        m_rightLauncherMotor.set(0);
    }

    // public void setLauncherVelocityTarget (double Speed) {
    //     m_leftLauncherMotor.set(-Speed);
    //     m_rightLauncherMotor.set(Speed);
    // }
}
