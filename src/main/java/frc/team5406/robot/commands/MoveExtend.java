// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class MoveExtend extends CommandBase
{

  private final ExtendSubsystem arm;
  private final DoubleSupplier position;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public MoveExtend(ExtendSubsystem arm, DoubleSupplier position)
  {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    arm.gotoExtendPosition(position.getAsDouble());
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
    if(Math.abs(arm.getExtendAbsolutePosition() - position.getAsDouble()) <0.5 && Math.abs(arm.getExtendVelocity()) < 1){
      return true;
    }else {
      return false;
    }  }
}