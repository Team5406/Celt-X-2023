// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem.LIMELIGHT_PIPELINE;

/**
 * An example command that uses an example subsystem.
 */
public class DriveToCubeNode extends CommandBase
{

  private final SwerveSubsystem drive;
  private final LimelightSubsystem limelight;
  private final DoubleSupplier vX;
  private PIDController forwardsController, sidewaysController, rotationController;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public DriveToCubeNode(SwerveSubsystem drive, LimelightSubsystem limelight, DoubleSupplier vX)
  {
    this.drive = drive;
    this.limelight = limelight;
    this.vX = vX;
    addRequirements(drive, limelight);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    forwardsController = new PIDController(0.1, 0, 0);
    sidewaysController = new PIDController(0.1, 0, 0);
    rotationController = new PIDController(0.15, 0, 0);
    rotationController.enableContinuousInput(-180, 180);
    rotationController.setSetpoint(180);
    forwardsController.setSetpoint(-7);
    sidewaysController.setSetpoint(0);
    rotationController.setTolerance(1);
    forwardsController.setTolerance(1);
    sidewaysController.setTolerance(0.3);

    limelight.setLLPipeline(LIMELIGHT_PIPELINE.APRILTAG);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    double fwd = -MathUtil.clamp(forwardsController.calculate(limelight.getLLty()), -4, 4);
    double side = MathUtil.clamp(sidewaysController.calculate(limelight.getLLtx()), -1, 1);
    double rot = MathUtil.clamp(rotationController.calculate(drive.getHeading().getDegrees()), -5, 5);
    
    
      drive.setChassisSpeeds(new ChassisSpeeds(fwd, side, rot)); 

    

    
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
    if(rotationController.atSetpoint() && sidewaysController.atSetpoint() && forwardsController.atSetpoint()){
      return true;
  }else{
      return false;
  }  
    }
}