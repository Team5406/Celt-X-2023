// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;

/**
 * An example command that uses an example subsystem.
 */
public class ExtendToL1 extends SequentialCommandGroup
{

  public ExtendToL1(ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake)
  {
    addCommands(
      new ParallelDeadlineGroup(
            new WaitCommand(0.25),
            new MoveWristTrapezoid(Constants.WRIST_OUTAKE_POSITION, wrist, shoulder, 9000)
      ),
      new ConditionalCommand(
            new RunCommand(() -> intake.cubeOuttake(), intake), 
            new RunCommand(() -> intake.coneOuttake(), intake),                 
            () -> intake.getGamePieceType() == GamePieces.CUBE
      )

    );
  }

 
}