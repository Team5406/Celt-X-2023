// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem.LIMELIGHT_PIPELINE;
import frc.team5406.robot.lib.swervelib.SwerveController;

/**
 * An example command that uses an example subsystem.
 */
public class DriveToConeNodeL2 extends CommandBase
{

  
  private final SwerveSubsystem drive;
  private final LimelightSubsystem limelight;
  private final DoubleSupplier vX;
  private PIDController sidewaysController, rotationController;
  private int speedMultiplier = 1;
  private final SwerveController controller;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public DriveToConeNodeL2(SwerveSubsystem drive, LimelightSubsystem limelight, DoubleSupplier vX)
  {
    this.drive = drive;
    this.limelight = limelight;
    this.vX = vX;
    this.controller = drive.getSwerveController();
    addRequirements(drive, limelight);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    speedMultiplier = DriverStation.getAlliance() == Alliance.Red ? 1 : -1;
    sidewaysController = new PIDController(0.1, 0, 0);
    rotationController = new PIDController(0.1, 0, 1e-5);
    rotationController.enableContinuousInput(-180, 180);
    rotationController.setSetpoint(180);
    sidewaysController.setSetpoint(0);
    rotationController.setTolerance(1);
    sidewaysController.setTolerance(0.3);
    limelight.turnOnLED();
    limelight.setLLPipeline(LIMELIGHT_PIPELINE.L2);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    double side = MathUtil.clamp(sidewaysController.calculate(limelight.getLLtx()), -1, 1);
    double rot = MathUtil.clamp(rotationController.calculate(drive.getHeading().getDegrees()), -0.5*controller.config.maxAngularVelocity, 0.5*controller.config.maxAngularVelocity);
    
    if(limelight.getLLtv()!=0){
      drive.setChassisSpeeds(new ChassisSpeeds(Math.pow(-vX.getAsDouble(), 3)*controller.config.maxSpeed, side, rot));

    }else{
      drive.setChassisSpeeds(new ChassisSpeeds(Math.pow(-vX.getAsDouble(), 3)*controller.config.maxSpeed, 0, 0));
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
    if(rotationController.atSetpoint() && sidewaysController.atSetpoint() && (Math.abs(vX.getAsDouble()) < 0.1)){
      return true;
  }else{
      return false;
  }  
    }
}