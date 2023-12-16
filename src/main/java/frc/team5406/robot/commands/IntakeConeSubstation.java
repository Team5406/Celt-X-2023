package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;

public class IntakeConeSubstation extends ParallelCommandGroup
{

  public IntakeConeSubstation(ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake)
  {
    addCommands(
      new InstantCommand(() -> intake.setHeadlights(true)),
      new MoveShoulderTrapezoid(1, extend, shoulder),
      new MoveWristTrapezoid(Constants.WRIST_SUBSTATION_CONE_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_SLOW_ACCELERATION),
      new InstantCommand(() -> intake.coneIntake(), intake)
);
  }

 
}
