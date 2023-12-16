package frc.team5406.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.team5406.robot.commands.AutoBalance;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;

public final class Balance
{

  /**
   * April Tag field layout.
   */

  private Balance()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve)
  {
   
    return Commands.sequence(
      new AutoBalance(swerve, true),
      new RepeatCommand(new InstantCommand(swerve::lock, swerve))
    );
  }

 
}