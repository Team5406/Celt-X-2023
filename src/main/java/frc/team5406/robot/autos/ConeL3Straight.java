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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.ExtendToL3;
import frc.team5406.robot.commands.ScoreGamepieceL3;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;

public final class ConeL3Straight
{

  /**
   * April Tag field layout.
   */
  private ConeL3Straight()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake, LimelightSubsystem limelight)
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
      new InstantCommand(() -> intake.setGamePieceType(GamePieces.CONE)),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CONE)),
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new ExtendToL3(shoulder, extend, wrist, limelight)
      ),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
      new ScoreGamepieceL3(shoulder, extend, wrist, intake),
      new SequentialCommandGroup(
        new WaitCommand(0.5),
        new InstantCommand(() -> swerve.resetOdometry(driveStraight.getInitialHolonomicPose())),
        new FollowTrajectory(swerve, driveStraight, true)
      )
      )
    ));
  }

 
}