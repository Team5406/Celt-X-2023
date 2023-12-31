package frc.team5406.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;

public class FollowTrajectory extends SequentialCommandGroup
{
  double currentAngle;

  public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry)
  {
    addRequirements(drivebase);

    if (resetOdometry)
    {
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    }

    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            Constants.xAutoPID.createPIDController(),
            Constants.yAutoPID.createPIDController(),
            Constants.angleAutoPID.createPIDController(),
            drivebase::setChassisSpeeds,
            false,
            drivebase)
               );
  }

  public FollowTrajectory(SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry, boolean holdAngle)
  {
    addRequirements(drivebase);
    
    if (resetOdometry)
    {
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    }

    addCommands(
        new ConditionalCommand(new InstantCommand(() -> drivebase.saveHeading()), new WaitCommand(0), () -> holdAngle),
        new ConditionalCommand(new InstantCommand(() -> drivebase.zeroGyro()), new WaitCommand(0), () -> holdAngle),
        new PPSwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            Constants.xAutoPID.createPIDController(),
            Constants.yAutoPID.createPIDController(),
            Constants.angleAutoPID.createPIDController(),
            drivebase::setChassisSpeeds,
            false,
            drivebase),
            new ConditionalCommand(new InstantCommand(() -> drivebase.restoreHeading()), new WaitCommand(0), () -> holdAngle)
        );
  }
}