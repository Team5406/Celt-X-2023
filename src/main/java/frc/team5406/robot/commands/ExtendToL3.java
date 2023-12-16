// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ExtendToL3 extends SequentialCommandGroup
{

  public ExtendToL3(ShoulderSubsystem shoulder, ExtendSubsystem extend, WristSubsystem wrist, LimelightSubsystem limelight)
  {
    addCommands(
          new InstantCommand(() -> extend.setArmLevelThree(), extend),
          new ParallelDeadlineGroup(
            new WaitCommand(0.05), 
            new MoveExtendTrapezoid(Constants.EXTEND_HOME_POSITION, extend, shoulder),
            new InstantCommand(() -> limelight.turnOffLED(), limelight)
            ),
            new ParallelCommandGroup(
              new MoveShoulderTrapezoid(77, extend, shoulder),
              new SequentialCommandGroup(
                      new WaitCommand(0.4),
                      new MoveExtendTrapezoid(7, extend, shoulder)),
              new SequentialCommandGroup(
                      new WaitCommand(0.4),
                      new MoveWristTrapezoid(170, wrist, shoulder, Constants.WRIST_EXTEND_SLOW_ACCELERATION))
            )
           
    );
  }

 
}