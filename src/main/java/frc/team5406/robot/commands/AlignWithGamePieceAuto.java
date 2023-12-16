package frc.team5406.robot.commands;

import frc.team5406.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.vision.ObjectDetectionSubsystem;



/** A command that will turn the robot to the specified angle. */
public class AlignWithGamePieceAuto extends PIDCommand {

  private final SwerveSubsystem swerve;
  private final ObjectDetectionSubsystem orangepi;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final boolean cone;

    //DriveSubsystem drive;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public AlignWithGamePieceAuto(SwerveSubsystem swerve, ObjectDetectionSubsystem orangepi, DoubleSupplier vX, DoubleSupplier vY, boolean cone) {
    super(
        new PIDController(1e-2, 0, 1e-5),
        // Close loop on heading
        () -> orangepi.getCenterDistance(cone),
        // center limelight on target, tx = 0
        0,
        // Pipe output to turn robot
        output -> swerve.drive(new Translation2d(Math.pow(vX.getAsDouble(), 3), Math.pow(vY.getAsDouble(), 3)), output, true, false),
        // Require the drive
        swerve);

    // Set the controller to be continuous (because it is an angle controller)
    //getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.LL_TURN_TOLERANCE);
        System.out.println("Align With Node - Start");
        this.swerve = swerve;
        this.orangepi = orangepi;
        this.vX = vX;
        this.vY = vY;
        this.cone = cone;



  }

  @Override
  public boolean isFinished() {
    //never end
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
      swerve.drive(new Translation2d(0, 0), 0, false, false);
      System.out.println("Align With Node - End");
  }
}