package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.AutonomousGrabberConveyorCmd;
import frc.robot.commands.AutonomousLauncherCmd;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class TwoNotesRedSpeaker extends SequentialCommandGroup {
  public TwoNotesRedSpeaker(Swerve s_Swerve, LauncherSubsystem launcherSubsystem, GrabberSubsystem grabberSubsystem, ConveyorSubsystem conveyorSubsystem) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            
            // Start at the origin facing the +X direction
            // S curve drive
            //new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            //new Pose2d(3, 0, new Rotation2d(0)),
            //config);

            // Start at the origin facing the +X direction
            // An example trajectory to go 1 meter forward and then turn 90 degrees
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior waypoints in this trajectory
            List.of(),
            // End 1 meter straight ahead of where we started, facing 90 degrees to the left (π/2 radians)
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

    Trajectory reverseTrajectory =
        TrajectoryGenerator.generateTrajectory(
            
            // Start at the origin facing the +X direction
            // S curve drive
            //new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            //new Pose2d(3, 0, new Rotation2d(0)),
            //config);

            // Start at the origin facing the +X direction
            // An example trajectory to go 1 meter forward and then turn 90 degrees
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior waypoints in this trajectory
            List.of(),
            // End 1 meter straight ahead of where we started, facing 90 degrees to the left (π/2 radians)
            new Pose2d(-2, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

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
        new AutonomousLauncherCmd(launcherSubsystem, 5000, conveyorSubsystem, 1.0, 3),
        new ParallelCommandGroup(
            new InstantCommand(() -> s_Swerve.setPose(forwardTrajectory.getInitialPose())),
            forwardSwerveControllerCommand,
            new AutonomousGrabberConveyorCmd(grabberSubsystem, 0, conveyorSubsystem, 0, null, 0)
        ),

        new ParallelCommandGroup(
            new InstantCommand(() -> s_Swerve.setPose(reverseTrajectory.getInitialPose())),
            reverseSwerveControllerCommand,
            new AutonomousGrabberConveyorCmd(grabberSubsystem, 0, conveyorSubsystem, 0, null, 0)
        ),

        new AutonomousLauncherCmd(launcherSubsystem, 5000, conveyorSubsystem, 1.0, 3)
        
        // new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
        // swerveControllerCommand //,
        );
  }
}