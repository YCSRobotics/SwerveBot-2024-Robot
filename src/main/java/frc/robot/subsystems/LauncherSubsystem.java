package frc.robot.subsystems;

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

public class LauncherSubsystem {
    private CANSparkMax m_leftLauncherMotor;
    private CANSparkMax m_rightLauncherMotor;

    public LauncherSubsystem(){
        m_leftLauncherMotor = new CANSparkMax(1, MotorType.kBrushless);
        m_rightLauncherMotor= new CANSparkMax(2, MotorType.kBrushless);
    }

    public void setLauncherMotorSpeed (double leftLauncherSpeed, double rightLauncherSpeed){
        m_leftLauncherMotor.set(leftLauncherSpeed);
        m_rightLauncherMotor.set(rightLauncherSpeed);
    }

    //private CANSparkMax m_flywheelMotorLeft = new CANSparkMax(0, MotorType.kBrushless);
    //private CANSparkMax m_flywheelMotorRight = new CANSparkMax(0, MotorType.kBrushless);
    //private SparkPIDController m_flywheelMotorLeftPidController;
    //private SparkPIDController m_flywheelMotorRightPidController;
}