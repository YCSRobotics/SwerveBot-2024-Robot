package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.AutonomousGrabberConveyorCmd;
import frc.robot.commands.AutonomousLauncherCmd;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ProximitySensorSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class Side_RedPoleBlueNo extends SequentialCommandGroup {
  public Side_RedPoleBlueNo(Swerve s_Swerve, LauncherSubsystem launcherSubsystem, GrabberSubsystem grabberSubsystem, ConveyorSubsystem conveyorSubsystem, ProximitySensorSubsystem proximitySensorSubsystem) {

    /*Trajectory Configurations*/
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond_Side,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics)
            .setReversed(false);

    TrajectoryConfig configReverse =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond_Side,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics)
            .setReversed(true);

    /*Trajectories, All units in meters*/
    Trajectory forwardCenterTrajectory =
        TrajectoryGenerator.generateTrajectory(

            new Pose2d(0, 0, new Rotation2d(Math.PI / 3)),
            // No interior waypoints in this trajectory
            List.of(), // 0.4
            // End 1 meter straight ahead of where we started, facing 90 degrees to the left (π/2 radians)
            new Pose2d(1.9, 0.1, new Rotation2d(Math.PI / 8)), // y: 0.4
            config);

    Trajectory reverseCenterTrajectory =
        TrajectoryGenerator.generateTrajectory(
            //orwardCenterTrajectory.getStates().get(forwardCenterTrajectory.getStates().size() - 1).poseMeters, // Start at the end of the forward trajectory
            new Pose2d(1.9, 0.1, new Rotation2d(Math.PI/ 8)),  //7
            List.of(),
            new Pose2d(0, 0, new Rotation2d(Math.PI / 3)),
            configReverse);

    Trajectory forwardPoleTrajectory =
        TrajectoryGenerator.generateTrajectory(

            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(0.6, 1.3)),
            new Pose2d(1.3, 1.3, new Rotation2d(0)),
            config);
            // new Translation2d(1, 1.2)

    Trajectory reversePoleTrajectory =
        TrajectoryGenerator.generateTrajectory(

            new Pose2d(1.3, 1.3, new Rotation2d(0)),
            List.of(),
            new Pose2d(0, 0, new Rotation2d(0)),
            configReverse);
            // new Translation2d(0.2, 0)

    /*Theta Controller*/
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    /*Swerve Controller Commands*/
    SwerveControllerCommand forwardCenterSwerveControllerCommand =
        new SwerveControllerCommand(
            forwardCenterTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand reverseCenterSwerveControllerCommand =
        new SwerveControllerCommand(
            reverseCenterTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand forwardPoleSwerveControllerCommand =
        new SwerveControllerCommand(
            forwardPoleTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    SwerveControllerCommand reversePoleSwerveControllerCommand =
        new SwerveControllerCommand(
            reversePoleTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new AutonomousLauncherCmd(launcherSubsystem, 5000, conveyorSubsystem, 0.60, 2),

        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.setPose(forwardCenterTrajectory.getInitialPose())),
                forwardCenterSwerveControllerCommand,
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false))
            ),
            new AutonomousGrabberConveyorCmd(grabberSubsystem, 0.60, conveyorSubsystem, 0.60, proximitySensorSubsystem, 3)
        ),

        new ParallelCommandGroup(
             new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.setPose(reverseCenterTrajectory.getInitialPose())),
                reverseCenterSwerveControllerCommand,
                new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false))
            ),
            new AutonomousGrabberConveyorCmd(grabberSubsystem, 0.60, conveyorSubsystem, 0.60, proximitySensorSubsystem, 2)
        ),

        new AutonomousLauncherCmd(launcherSubsystem, 5000, conveyorSubsystem, 0.60, 2)

        // new ParallelCommandGroup(
        //     new SequentialCommandGroup(
        //         new InstantCommand(() -> s_Swerve.setPose(forwardPoleTrajectory.getInitialPose())),
        //         forwardPoleSwerveControllerCommand,
        //         new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false))
        //     ),
        //     new AutonomousGrabberConveyorCmd(grabberSubsystem, 0.60, conveyorSubsystem, 0.60, proximitySensorSubsystem, 3)
        // ),

        // new ParallelCommandGroup(
        //      new SequentialCommandGroup(
        //         new InstantCommand(() -> s_Swerve.setPose(reversePoleTrajectory.getInitialPose())),
        //         reversePoleSwerveControllerCommand,
        //         new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, false, false))
        //     ),
        //     new AutonomousGrabberConveyorCmd(grabberSubsystem, 0.60, conveyorSubsystem, 0.60, proximitySensorSubsystem, 2)
        // ),

        // new AutonomousLauncherCmd(launcherSubsystem, 5000, conveyorSubsystem, 0.60, 2)        
        );
  }
}