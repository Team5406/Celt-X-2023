// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class MoveWrist extends CommandBase
{

  private final WristSubsystem arm;
  private final DoubleSupplier angle;
  private final DoubleSupplier tolerance;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public MoveWrist(WristSubsystem arm, DoubleSupplier angle, DoubleSupplier tolerance)
  {
    this.arm = arm;
    this.angle = angle;
    this.tolerance = tolerance;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    arm.gotoWristAngle(angle.getAsDouble());
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

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if(Math.abs(arm.getWristAbsoluteEncoderPosition() - angle.getAsDouble()) <tolerance.getAsDouble() && Math.abs(arm.getWristVelocity()) < tolerance.getAsDouble()*5){
      return true;
    }else {
      return false;
    }  }
}