// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ScoreGamepieceL3 extends SequentialCommandGroup
{

  public ScoreGamepieceL3(ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake)
  {
    addCommands(
      new ConditionalCommand(
        new ParallelDeadlineGroup(
          new WaitCommand(0.5),
          new RunCommand(() -> intake.cubeOuttake(), intake)

        ),
        new WaitCommand(0),
        () -> intake.getGamePieceType() == GamePieces.CUBE

      ),

     new ParallelCommandGroup(
      new InstantCommand(() -> intake.stopHoldingCurrent(), intake),
      new ConditionalCommand(
        new ParallelDeadlineGroup(
          new WaitCommand(0.4),
          new SequentialCommandGroup(
            new WaitCommand(0.06),
            new MoveWristTrapezoidRelative(7, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION)
          )
        ),
        new WaitCommand(0),
        () -> intake.getGamePieceType() == GamePieces.CONE
      ),
      new SequentialCommandGroup(
        new WaitCommand(0.0),
        new MoveExtendTrapezoid(Constants.EXTEND_HOME_POSITION, extend, shoulder)
      )
     ) ,
      new ParallelCommandGroup(
        new MoveShoulderTrapezoid(8, extend, shoulder),
        new MoveWristTrapezoid(6, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION),
        new InstantCommand(() -> intake.stopHoldingCurrent(), intake))

    );
  }


 
}