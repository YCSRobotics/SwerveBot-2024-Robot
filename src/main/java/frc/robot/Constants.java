package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(24.125);
    public static final double wheelBase = Units.inchesToMeters(24.1875);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 6.75:1
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 0;
      //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(173.145);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(352.441);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(263.320);
      //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(84.639);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 2;
      //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(5.098);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(184.922);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 3;
      //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(74.268);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(254.180);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class Mechanisms {

    /*CAN IDs*/
    public static final int leftLauncherMotorID = 9;
    public static final int rightLauncherMotorID = 10;

    public static final int grabberMotorID = 11;
    public static final int conveyorMotorID = 12;

    public static final int leftHangerMotorID = 13;
    public static final int rightHangerMotorID = 14;

    public static final int flipMotorID = 16;
    public static final int rotateMotorID = 15; 


    /*Target Speeds*/
    // public static final double launcherTargetSpeed = 0.25; // Was 1.0
    public static final double grabberTargetSpeed = 0.6;
    public static final double conveyorTargetSpeed = 1.0;
    public static final double ampVelocityTargetSpeed = 250;

    public static final double launcherVelocityTarget = 5000.0; // 250 for Amp

    // GrabberConveyor - Reverse Target Speeds
    public static final double reversegrabberTargetSpeed = -0.6;
    public static final double reverseconveyorTargetSpeed = -1.0;

    // Trap - Target Speeds
    public static final double trapArmRotateLeftSpeed = -0.05;
    public static final double trapArmRotateRightSpeed = 0.05;

    public static final double trapArmFlipLeftSpeed = 0.0;//??
    public static final double trapArmFlipRightSpeed = 0.0;//??


    public static final double trapArmGrabSpeed = 0.0;//??
    public static final double trapArmReleaseSpeed = 0.0;//??


    /*Hanger Target Position*/
    public static final double hangerTargetPosition = 50.0;


    /*Current Limits*/
    public static final int launcherCurrentLimit = 60;
    public static final int grabberCurrentLimit = 30;
    public static final int conveyorCurrentLimit = 30;
    public static final int hangerCurrentLimit = 40;


    /*Hanger*/
    public static final double hangerPIDControllerkP = 0.0001; // Originally 0.05
    public static final double hangerPIDControllerkI = 0.0; // Originally 0.00002
    public static final double hangerPIDControllerkD = 0.0;

    public static DigitalInput leftLimitSwitch = new DigitalInput(0);
    public static DigitalInput rightLimitSwitch = new DigitalInput(1);

    public static final double upVelocity = 3000;
    public static final double downVelocity = -3000;


    /*Launcher*/
    public static final double launcherPIDControllerkP = 0.0004;
    public static final double launcherPIDControllerkI = 0.000001;
    public static final double launcherPIDControllerkD = 0.0;

     /*TrapArm*/
    public static final double rotateMaxLimit = 25;//??
    public static final double rotateMinLimit = 0;//??
    
    public static final double flipMaxLimit = 25;//??
    public static final double flipMinLimit = -25;//??

    public static final int solenoidPort = 15;
   
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}

//test comment 1/25/24 3:58