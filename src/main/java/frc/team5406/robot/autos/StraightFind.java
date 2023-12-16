package frc.team5406.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AlignWithGamePieceAuto;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.ObjectDetectionSubsystem;

public final class StraightFind
{

  /**
   * April Tag field layout.
   */
  private StraightFind()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake, LimelightSubsystem limelight, ObjectDetectionSubsystem orangePi)
  {
    boolean blue = false;
    PathPlannerTrajectory driveToCube;
    PathPlannerTrajectory pickUpCube;
    PathPlannerTrajectory driveToChargeStation;
    double multiplier = blue?-1:1;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveToCube = PathPlanner.generatePath(
          new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
          true,
          new PathPoint(new Translation2d(0, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
          new PathPoint(new Translation2d(-1, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*180))
      );

      
    
    return Commands.sequence(new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new FollowTrajectory(swerve, driveToCube, true)
        //new IntakeCubeFloor(shoulder, extend, wrist, intake)
      ),
      new PrintCommand("Auto - Line 2"),
      new ParallelRaceGroup(
        new WaitCommand(0.5), 
        new RepeatCommand(
          new AlignWithGamePieceAuto(swerve, orangePi, () -> 0, () -> 0, false)
        )
      ),
      Drive1m(swerve),
      //new InstantCommand(() -> swerve.setGyro(DriverStation.getAlliance()==Alliance.Red?180:0)),
      new RepeatCommand(new InstantCommand(swerve::lock, swerve))

    ));
  }

  public static Command Drive1m(SwerveSubsystem drive){
    return new ProxyCommand(() -> {
      Pose2d pose = drive.getPose();
    Translation2d translation = pose.getTranslation();
     Translation2d destination = translation.plus(new Translation2d(1, pose.getRotation()));

    System.out.println("Pose: " + pose.getX() + ", " + pose.getY());
    System.out.println("Translation: " + translation.getX() + ", " + translation.getY());
    System.out.println("Destination: " + destination.getX() + ", " + translation.getY());
    System.out.println("Rotation: " + pose.getRotation());
    System.out.println("Heading: " + drive.getHeading());

    
    
    // More complex path with holonomic rotation. Non-zero starting velocity of 2 m/s. Max velocity of 4 m/s and max accel of 3 m/s^2
PathPlannerTrajectory traj = PathPlanner.generatePath(
    new PathConstraints(2.5, 2), 
    new PathPoint(drive.getPose().getTranslation(),drive.getHeading(), drive.getPose().getRotation()),
    new PathPoint(destination,drive.getHeading(), drive.getPose().getRotation())
 );

//FieldTrajectoryConstants.fieldLengthMeters - 1.2. FieldTrajectoryConstants.fieldWidthMeters - 0.6
  return  new FollowTrajectory(drive, traj, false);
    }
    );
    

}

 
}