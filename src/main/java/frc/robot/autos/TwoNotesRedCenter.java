package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.AutonomousGrabberConveyorCmd;
import frc.robot.commands.AutonomousLauncherCmd;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ProximitySensorSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class TwoNotesRedCenter extends SequentialCommandGroup {
  public TwoNotesRedCenter(Swerve s_Swerve, LauncherSubsystem launcherSubsystem, GrabberSubsystem grabberSubsystem, ConveyorSubsystem conveyorSubsystem, ProximitySensorSubsystem proximitySensorSubsystem) {
    
    /*Trajectory Configurations*/
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics)
            .setReversed(false);

    TrajectoryConfig configReverse =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics)
            .setReversed(true);

    /*Trajectories, All units in meters*/
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 0)),
            new Pose2d(1.2, 0, new Rotation2d(0)), // x was 2
            config);

    Trajectory reverseTrajectory =
        TrajectoryGenerator.generateTrajectory(
            
            new Pose2d(1.2, 0, new Rotation2d(0)), // x was 2
            List.of(new Translation2d(1, 0)),
            new Pose2d(0, 0, new Rotation2d(0)),
            configReverse);

    /*Theta Controller*/
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    /*Swerve Controller Commands*/
    SwerveControllerCommand forwardSwerveControllerCommand =
        new SwerveControllerCommand(
            forwardTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand reverseSwerveControllerCommand =
        new SwerveControllerCommand(
            reverseTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new AutonomousLauncherCmd(launcherSubsystem, 5000, conveyorSubsystem, 0.25, 2),
        
        new ParallelCommandGroup(

            new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.setPose(forwardTrajectory.getInitialPose())),
                forwardSwerveControllerCommand,
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false))
            ),

            new AutonomousGrabberConveyorCmd(grabberSubsystem, 0.60, conveyorSubsystem, 0.40, proximitySensorSubsystem, 3)
        
        ),

        new ParallelCommandGroup(
            
             new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.setPose(reverseTrajectory.getInitialPose())),
                reverseSwerveControllerCommand,
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false))
            ),

            new AutonomousGrabberConveyorCmd(grabberSubsystem, 0.60, conveyorSubsystem, 0.40, proximitySensorSubsystem, 2)
        ),

        new AutonomousLauncherCmd(launcherSubsystem, 5000, conveyorSubsystem, 0.25, 2)

        );
  }
}