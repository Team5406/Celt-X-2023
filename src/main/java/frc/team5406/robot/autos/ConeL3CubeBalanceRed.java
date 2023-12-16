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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AlignWithGamePieceAuto;
import frc.team5406.robot.commands.AutoBalance;
import frc.team5406.robot.commands.CrossChargeStation;
import frc.team5406.robot.commands.ExtendToL3;
import frc.team5406.robot.commands.IntakeCubeFloor;
import frc.team5406.robot.commands.MoveWristTrapezoid;
import frc.team5406.robot.commands.ScoreGamepieceL3;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.ObjectDetectionSubsystem;

public final class ConeL3CubeBalanceRed
{

  /**
   * April Tag field layout.
   */
  private ConeL3CubeBalanceRed()
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
    double multiplier = blue?-1:1;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveToCube = PathPlanner.generatePath(
          new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
          true,
          new PathPoint(new Translation2d(0, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*355)),
          new PathPoint(new Translation2d(-.3, multiplier*-.2), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*180))
      );

      
    
    return Commands.sequence(new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
      new InstantCommand(() -> intake.setGamePieceType(GamePieces.CUBE)),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new ExtendToL3(shoulder, extend, wrist, limelight)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.7),
          new CrossChargeStation(swerve, true)
        ),
        new ScoreGamepieceL3(shoulder, extend, wrist, intake)       
      ),
      new InstantCommand(() -> intake.setGamePieceType(GamePieces.CUBE)),
      new PrintCommand("Auto - Line 1"),
      resetOdometry(swerve),
      new ParallelDeadlineGroup(
        new FollowTrajectory(swerve, driveToCube, false),
        new IntakeCubeFloor(shoulder, extend, wrist, intake)
      ),
      new PrintCommand("Auto - Line 2"),
      new ParallelRaceGroup(
        new WaitCommand(0.5), 
        new RepeatCommand(
          new AlignWithGamePieceAuto(swerve, orangePi, () -> 0, () -> 0, false)
        )
      ),
      SecondHalf(swerve, shoulder, extend, wrist, intake, limelight, orangePi)
      
    ));
  }


  public static Command resetOdometry(SwerveSubsystem swerve){
    return new ProxyCommand(() -> {
      return new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0,0, swerve.getPose().getRotation())));
    }
    );
  }

  public static Command SecondHalf(SwerveSubsystem swerve, ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake, LimelightSubsystem limelight, ObjectDetectionSubsystem orangePi){
    return new ProxyCommand(() -> {
      Pose2d pose = swerve.getPose();
    Translation2d translation = pose.getTranslation();
    Translation2d destination = translation.plus(new Translation2d(1.1, pose.getRotation()));
    Translation2d destination2 = new Translation2d(-0.3,0);

    System.out.println("Pose: " + pose.getX() + ", " + pose.getY());
    System.out.println("Translation: " + translation.getX() + ", " + translation.getY());
    System.out.println("Destination: " + destination.getX() + ", " + translation.getY());
    System.out.println("Rotation: " + pose.getRotation());
    System.out.println("Heading: " + swerve.getHeading());

    
    
    // More complex path with holonomic rotation. Non-zero starting velocity of 2 m/s. Max velocity of 4 m/s and max accel of 3 m/s^2
PathPlannerTrajectory traj = PathPlanner.generatePath(
    new PathConstraints(2.5, 2.5), 
    new PathPoint(swerve.getPose().getTranslation(),swerve.getHeading(), swerve.getPose().getRotation()),
    new PathPoint(destination,swerve.getHeading(), swerve.getPose().getRotation())
 );

 PathPlannerTrajectory traj2 = PathPlanner.generatePath(
  new PathConstraints(2.5, 2.5), 
  true,
  new PathPoint(destination,swerve.getHeading().minus(Rotation2d.fromDegrees(180)), swerve.getPose().getRotation()),
  new PathPoint(destination2,Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 1.7)
);


  return new SequentialCommandGroup(
    
  new PrintCommand("Auto - Line 3"),
  
  new ParallelDeadlineGroup(
    new FollowTrajectory(swerve, traj, false),
    new IntakeCubeFloor(shoulder, extend, wrist, intake)
  ),

    new PrintCommand("Auto - Line 4"),

  new ParallelDeadlineGroup(
    new FollowTrajectory(swerve, traj2, false),
    new MoveWristTrapezoid(Constants.WRIST_HOME_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION),
    new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE))
  ),
  new AutoBalance(swerve, false),
  new RepeatCommand(new InstantCommand(swerve::lock, swerve))


  );

    }
    );
    

}


}