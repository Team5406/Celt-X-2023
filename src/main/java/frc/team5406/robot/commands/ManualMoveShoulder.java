// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ManualMoveShoulder extends CommandBase
{

  private final ShoulderSubsystem shoulder;
  private final DoubleSupplier angle;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public ManualMoveShoulder(ShoulderSubsystem shoulder, DoubleSupplier angle)
  {
    this.shoulder = shoulder;
    this.angle = angle;
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    shoulder.gotoShoulderAngle(angle.getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    shoulder.gotoShoulderAngle(shoulder.getShoulderAbsoluteEncoderPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
      return true;
    }
}