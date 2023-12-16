package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.Constants;
import frc.team5406.robot.lib.swervelib.SwerveController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;



/** A command that will turn the robot to the specified angle. */
public class AlignWithDirection extends CommandBase{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private DoubleSupplier angle;
  private PIDController rotationController;
  private final SwerveController controller;

    //DriveSubsystem drive;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public AlignWithDirection(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier angle) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.angle = angle;
        this.controller = swerve.getSwerveController();
        addRequirements(swerve);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
      System.out.println("Align With Direction - Start");
      rotationController = new PIDController(0.1, 0, 1e-5); 
      rotationController.enableContinuousInput(-180, 180);
      rotationController.setSetpoint(angle.getAsDouble());
      rotationController.setTolerance(Constants.ALIGN_WITH_DIRECTION_ROTATION_TOLERANCE);
    }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute(){
     
     swerve.drive(new Translation2d(Math.pow(vX.getAsDouble(), 3)*controller.config.maxSpeed, Math.pow(vY.getAsDouble(), 3)*controller.config.maxSpeed), MathUtil.clamp(rotationController.calculate(swerve.getHeading().getDegrees()), -controller.config.maxAngularVelocity, controller.config.maxAngularVelocity), true, false);
     SmartDashboard.putNumber("vX", vX.getAsDouble());
     SmartDashboard.putNumber("vY", vY.getAsDouble());
 
   }

  @Override
  public boolean isFinished() {
    //never end
    return false;
  }



  @Override
  public void end(boolean interrupted) {
      //swerve.drive(new Translation2d(0, 0), 0, false, false);
      System.out.println("Align With Direction - End");
      System.out.println("Align With Direction - Interrupted - " + interrupted);
  }
}