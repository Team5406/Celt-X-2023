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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AlignWithGamePieceAuto;
import frc.team5406.robot.commands.ExtendToL3;
import frc.team5406.robot.commands.IntakeCube;
import frc.team5406.robot.commands.ScoreGamepieceL3;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.ObjectDetectionSubsystem;

public final class ConeCubeEdgeBlue
{

  /**
   * April Tag field layout.
   */
  private ConeCubeEdgeBlue()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake, LimelightSubsystem limelight, ObjectDetectionSubsystem orangePi)
  {
    boolean blue = true;
    double multiplier = blue?-1:1;

    PathPlannerTrajectory driveToCube;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveToCube = PathPlanner.generatePath(
          new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND*0.8, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*0.8),
          true,
          new PathPoint(new Translation2d(0, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
          new PathPoint(new Translation2d(-2.7, multiplier*0.05), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*-5)),
          new PathPoint(new Translation2d(-4.0, multiplier*0.25), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*-178))
      );


      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
      // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
      //swerve.postTrajectory(driveToCube);
      //swerve.postTrajectory(driveToGrid);
      return Commands.sequence(new SequentialCommandGroup(
        new InstantCommand(() -> swerve.resetOdometry(driveToCube.getInitialHolonomicPose())),
      new InstantCommand(() -> intake.setGamePieceType(GamePieces.CONE)),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CONE)),
      new ParallelRaceGroup(
        new WaitCommand(2),
        new ExtendToL3(shoulder, extend, wrist, limelight)
      ),
      new ParallelRaceGroup(
        new WaitCommand(1.5),
        new ScoreGamepieceL3(shoulder, extend, wrist, intake)
      ),
      new FollowTrajectory(swerve, driveToCube, true),
      new ParallelRaceGroup(
        new WaitCommand(0.4), 
        new RepeatCommand(
          new AlignWithGamePieceAuto(swerve, orangePi, () -> 0, () -> 0, false)
        )
      ),
      SecondHalf(swerve, shoulder, extend, wrist, intake, limelight, orangePi)
      ));
      
  }

 

  public static Command SecondHalf(SwerveSubsystem swerve, ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake, LimelightSubsystem limelight, ObjectDetectionSubsystem orangePi){
    return new ProxyCommand(() -> {

      boolean blue = true;
      double multiplier = blue?-1:1;
  
  
      Pose2d pose = swerve.getPose();
    Translation2d translation = pose.getTranslation();
    Translation2d destination = translation.plus(new Translation2d(1.1, pose.getRotation()));

    System.out.println("Pose: " + pose.getX() + ", " + pose.getY());
    System.out.println("Translation: " + translation.getX() + ", " + translation.getY());
    System.out.println("Destination: " + destination.getX() + ", " + translation.getY());
    System.out.println("Rotation: " + pose.getRotation());
    System.out.println("Heading: " + swerve.getHeading());

    
    
    // More complex path with holonomic rotation. Non-zero starting velocity of 2 m/s. Max velocity of 4 m/s and max accel of 3 m/s^2
PathPlannerTrajectory traj = PathPlanner.generatePath(
    new PathConstraints(2.5, 2), 
    new PathPoint(swerve.getPose().getTranslation(),swerve.getHeading(), swerve.getPose().getRotation()),
    new PathPoint(destination,swerve.getHeading(), swerve.getPose().getRotation())
 );

 PathPlannerTrajectory driveToGrid = PathPlanner.generatePath(
  new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND*0.8, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED*0.9),
  true,
  new PathPoint(swerve.getPose().getTranslation(),swerve.getHeading(), swerve.getPose().getRotation()),
  new PathPoint(new Translation2d(-3, multiplier*0.1), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*5)),
  new PathPoint(new Translation2d(-0.15,multiplier* 0.9), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*0)),
  new PathPoint(new Translation2d(0.1,multiplier* 0.9), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*0))
);




  return new SequentialCommandGroup(

  new SequentialCommandGroup(
    new ParallelDeadlineGroup(
      new FollowTrajectory(swerve, traj, false),
      new IntakeCube(shoulder, extend, wrist, intake)
    ),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)),
      new InstantCommand(() -> wrist.gotoWristAngle(Constants.WRIST_HOME_POSITION), wrist),
      new ParallelCommandGroup(
        new FollowTrajectory(swerve, driveToGrid, false),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new ParallelRaceGroup(
            new WaitCommand(2),
            new ExtendToL3(shoulder, extend, wrist, limelight)
          )
        )
      ),
      new ParallelRaceGroup(
          new WaitCommand(1.5),
          new ScoreGamepieceL3(shoulder, extend, wrist, intake)
      )
    ));

  

    }
    );
    

}



}