package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase{
    public CANSparkMax m_leftLauncherMotor;
    public CANSparkMax m_rightLauncherMotor;

    public LauncherSubsystem(){
        m_leftLauncherMotor = new CANSparkMax(9, MotorType.kBrushed);
        m_rightLauncherMotor= new CANSparkMax(10, MotorType.kBrushed);

        m_leftLauncherMotor.setSmartCurrentLimit(30);
        m_rightLauncherMotor.setSmartCurrentLimit(30);
    }

    public void setLauncherTargetSpeed (double launcherTargetSpeed){
        m_leftLauncherMotor.set(launcherTargetSpeed);
        m_rightLauncherMotor.set(launcherTargetSpeed);
    }
}
