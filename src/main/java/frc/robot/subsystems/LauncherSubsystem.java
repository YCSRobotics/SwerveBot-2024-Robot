package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase{
    public CANSparkMax m_leftLauncherMotor;
    public CANSparkMax m_rightLauncherMotor;

    public LauncherSubsystem(){
        m_leftLauncherMotor = new CANSparkMax(Constants.Mechanisms.leftLauncherMotorID, MotorType.kBrushed);
        m_rightLauncherMotor = new CANSparkMax(Constants.Mechanisms.rightLauncherMotorID, MotorType.kBrushed);
        m_leftLauncherMotor.setSmartCurrentLimit(Constants.Mechanisms.launcherCurrentLimit);
        m_rightLauncherMotor.setSmartCurrentLimit(Constants.Mechanisms.launcherCurrentLimit);
    }

    public void setLauncherTargetSpeed (double Speed){
        m_leftLauncherMotor.set(-Speed);
        m_rightLauncherMotor.set(Speed);
    }
}
