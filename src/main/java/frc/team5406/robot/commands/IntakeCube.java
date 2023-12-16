// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeCube extends ParallelCommandGroup
{

  public IntakeCube(ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, IntakeSubsystem intake)
  {
    addCommands(
      new MoveShoulderTrapezoid(1, extend, shoulder),
      new MoveWristTrapezoid(Constants.WRIST_CUBE_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_SLOW_ACCELERATION),
      new InstantCommand(() -> intake.cubeIntake(), intake)
);
  }

 
}