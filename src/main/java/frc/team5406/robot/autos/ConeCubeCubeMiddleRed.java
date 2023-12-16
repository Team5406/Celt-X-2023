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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.ExtendToL3;
import frc.team5406.robot.commands.IntakeConeFloor;
import frc.team5406.robot.commands.IntakeCube;
import frc.team5406.robot.commands.ScoreGamepieceL3;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;

public final class ConeCubeCubeMiddleRed
{

  /**
   * April Tag field layout.
   */
  private ConeCubeCubeMiddleRed()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake, LimelightSubsystem limelight)
  {
    boolean blue = false;
    double multiplier = blue?-1:1;

    PathPlannerTrajectory driveToCube;
    PathPlannerTrajectory driveToGrid;
    PathPlannerTrajectory driveToCube2;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveToCube = PathPlanner.generatePath(
        new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
        new PathPoint(new Translation2d(0, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
        new PathPoint(new Translation2d(-2, multiplier*0.1), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
        new PathPoint(new Translation2d(-4.4, multiplier*-0.25), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*170)),
        new PathPoint(new Translation2d(-4.55, multiplier*-0.25), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*170)),
        new PathPoint(new Translation2d(-4.80, multiplier*-0.25), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*170))
    );
    driveToGrid = PathPlanner.generatePath(
      new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
      true,
      new PathPoint(new Translation2d(-4.85, multiplier*-0.25), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*-170)),
      new PathPoint(new Translation2d(-1.5, multiplier*-0.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*-5)),
      new PathPoint(new Translation2d(-0.15, multiplier*-0.85), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*-5))
  );

    driveToCube2 = PathPlanner.generatePath(
      new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
      true,
      new PathPoint(new Translation2d(-0.15, multiplier*-0.85), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
      new PathPoint(new Translation2d(-3.2, multiplier*-0.4), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
      new PathPoint(new Translation2d(-4.50, multiplier*-1.40), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*-150)),
      new PathPoint(new Translation2d(-4.85, multiplier*-1.60), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*-150))
  );


      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
      // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
      //swerve.postTrajectory(driveToCube);
      //swerve.postTrajectory(driveToGrid);
      return Commands.sequence(new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometry(driveToCube.getInitialHolonomicPose())),
      new ParallelRaceGroup(
        new WaitCommand(1.75),
        new ExtendToL3(shoulder, extend, wrist, limelight)
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FollowTrajectory(swerve, driveToCube, true)
        ),
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(2),
            new ScoreGamepieceL3(shoulder, extend, wrist, intake)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new IntakeCube(shoulder, extend, wrist, intake)
          )
        )
      ),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)),
      new InstantCommand(() -> wrist.gotoWristAngle(Constants.WRIST_HOME_POSITION), wrist), 
      new ParallelCommandGroup(
        new FollowTrajectory(swerve, driveToGrid, false),
        new SequentialCommandGroup(
          new WaitCommand(2),
          new ParallelRaceGroup(
            new WaitCommand(1.5),
            new ExtendToL3(shoulder, extend, wrist, limelight)
          )
        )
      ),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FollowTrajectory(swerve, driveToCube2, false)
        ),
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(2),
            new ScoreGamepieceL3(shoulder, extend, wrist, intake)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new IntakeConeFloor(shoulder, extend, wrist, intake)
          )
        )
      ),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CONE)),
      new InstantCommand(() -> wrist.gotoWristAngle(Constants.WRIST_HOME_POSITION), wrist), 
      new InstantCommand(() -> intake.setGamePieceType(GamePieces.CONE), intake), 
      new WaitCommand(0.5),
      new RepeatCommand(new InstantCommand(swerve::lock, swerve))
    ));
  }

 
}