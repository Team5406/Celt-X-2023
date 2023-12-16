// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class CancelScoringGamepiece extends SequentialCommandGroup
{

  public CancelScoringGamepiece(ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist)
  {
    addCommands(
      new ParallelCommandGroup(
        new MoveWristTrapezoid(6, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION),

      new SequentialCommandGroup(
              new WaitCommand(0.2),
              new MoveExtendTrapezoid(Constants.EXTEND_HOME_POSITION, extend, shoulder)
      ),

      new SequentialCommandGroup(
              new WaitCommand(0.5),
              new MoveShoulderTrapezoid(5, extend, shoulder)
      )
      )

      

    );
  }

 
}