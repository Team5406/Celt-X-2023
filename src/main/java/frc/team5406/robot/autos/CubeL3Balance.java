package frc.team5406.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AutoBalanceOld;
import frc.team5406.robot.commands.ExtendToL3;
import frc.team5406.robot.commands.ScoreGamepieceL3;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;

public final class CubeL3Balance
{

  /**
   * April Tag field layout.
   */
  private CubeL3Balance()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake, LimelightSubsystem limelight)
  {
    
    return Commands.sequence(new SequentialCommandGroup(
      new InstantCommand(() -> intake.setGamePieceType(GamePieces.CUBE)),
      new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)),
      new ParallelDeadlineGroup(
        new WaitCommand(1.25),
        new ExtendToL3(shoulder, extend, wrist, limelight)
      ),
      new ParallelCommandGroup(
      new ScoreGamepieceL3(shoulder, extend, wrist, intake),
      new SequentialCommandGroup(
        new AutoBalanceOld(swerve, true)
        )
      ),
      new RepeatCommand(new InstantCommand(swerve::lock, swerve))

    ));
  }

 
}