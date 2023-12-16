package frc.team5406.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;

public final class DriveStraight{

  /**
   * April Tag field layout.
   */
  private DriveStraight()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve)
  {
    PathPlannerTrajectory driveStraight;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveStraight = PathPlanner.generatePath(
        new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0)),
// position, heading(direction of travel), holonomic rotation
          new PathPoint(new Translation2d(-4.5, 0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(0))
// position, heading(direction of travel), holonomic rotation
                                        );

      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
      // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
   swerve.postTrajectory(driveStraight);
    return Commands.sequence(new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(driveStraight.getInitialHolonomicPose())),
        new FollowTrajectory(swerve, driveStraight, true)

    ));
  }

 
}