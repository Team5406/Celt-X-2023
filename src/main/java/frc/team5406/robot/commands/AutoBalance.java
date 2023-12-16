// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import frc.team5406.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class AutoBalance extends CommandBase
{

  private final SwerveSubsystem drive;
  private Boolean backwards = false;
  boolean hasRisen = false;
  boolean hasFallen = false;
  double startAngle;
  double currentAngle;
  double startX;
  int speedMultiplier = 1;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public AutoBalance(SwerveSubsystem drive)
  {
    this.drive = drive;
    addRequirements(drive);
  }

  public AutoBalance(SwerveSubsystem drive, Boolean backwards)
  {
    this.drive = drive;
    this.backwards = backwards;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    hasRisen = false;
    hasFallen = false;
    startAngle = 360; //drive.getPitch().getDegrees();
    speedMultiplier = backwards ? -1 : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    currentAngle = drive.getPitch().getDegrees() - startAngle;  //FIXME
    System.out.println(hasRisen);
    System.out.println(currentAngle);
    if(Math.abs(currentAngle) > Constants.DRIVE_CLIMB_RISING_THRESHOLD){
        System.out.println("Print 1");
        hasRisen = true;
    }

    if(hasRisen && Math.abs(currentAngle) < Constants.DRIVE_CLIMB_LEVEL_THRESHOLD){
      hasFallen = true;
      startX = drive.getPose().getX();
    } 

    if(hasFallen){
      double speed = speedMultiplier*1.5;
      drive.setChassisSpeeds(new ChassisSpeeds(-speed, 0, 0));

    }else{
      drive.setChassisSpeeds(new ChassisSpeeds(speedMultiplier * Constants.DRIVE_CLIMB_FAST_SPEED, 0, 0));

    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    drive.lock();
    System.out.println("Print 6");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    if(hasFallen && startX - drive.getPose().getX() < -0.6){
      System.out.println("Print 4");
      return true;
  }else{
      return false;
  }  
    }
}