package frc.team5406.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AutoBalanceOld;
import frc.team5406.robot.commands.CrossChargeStation;
import frc.team5406.robot.commands.ExtendToL3;
import frc.team5406.robot.commands.ScoreGamepieceL3;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.ObjectDetectionSubsystem;

public final class L3CubeMobilityBalance
{

  /**
   * April Tag field layout.
   */
  private L3CubeMobilityBalance()
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
          new PathPoint(new Translation2d(0, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*340)),
          new PathPoint(new Translation2d(-.2, multiplier*-.3), Rotation2d.fromDegrees(220), Rotation2d.fromDegrees(multiplier*180))
      );
     pickUpCube = PathPlanner.generatePath(
          new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
          new PathPoint(new Translation2d(-.2, multiplier*-.3), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*0)),
          new PathPoint(new Translation2d(.8, multiplier*-.3), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*0))
      );
      driveToChargeStation = PathPlanner.generatePath(
        new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
        true,
        new PathPoint(new Translation2d(0.8, multiplier*-0.3), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
        new PathPoint(new Translation2d(-0.4, multiplier*-0.3), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*160))
    );
      
    
    return Commands.sequence(new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
      new InstantCommand(() -> intake.setGamePieceType(GamePieces.CUBE)),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)),
      new ParallelDeadlineGroup(
        new WaitCommand(1.25),
        new ExtendToL3(shoulder, extend, wrist, limelight)
      ),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.75),
          new CrossChargeStation(swerve, true)
        ),
        new ScoreGamepieceL3(shoulder, extend, wrist, intake)       
      ),
      new WaitCommand(2),
      new AutoBalanceOld(swerve, false),
      new RepeatCommand(new InstantCommand(swerve::lock, swerve))

    ));
  }

 
}