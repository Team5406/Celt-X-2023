package frc.team5406.robot.commands;

import frc.team5406.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem.LIMELIGHT_PIPELINE;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;



/** A command that will turn the robot to the specified angle. */
public class AlignWithNodeAuto extends PIDCommand {

  private final SwerveSubsystem swerve;
  private final LimelightSubsystem limelight;
  private final DoubleSupplier vX;
  private final DoubleSupplier vY;
  private final LIMELIGHT_PIPELINE pipeline;

    //DriveSubsystem drive;
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public AlignWithNodeAuto(SwerveSubsystem swerve, LimelightSubsystem limelight, DoubleSupplier vX, DoubleSupplier vY, LIMELIGHT_PIPELINE pipeline) {
    
    super(
        new PIDController(Constants.LL_TURN_P, Constants.LL_TURN_I, Constants.LL_TURN_D),
        // Close loop on heading
        limelight::getLLtx,
        // center limelight on target, tx = 0
        0,
        // Pipe output to turn robot
        output -> swerve.drive(new Translation2d(vX.getAsDouble(), vY.getAsDouble()), output, true, false),
        // Require the drive
        swerve, limelight);

    // Set the controller to be continuous (because it is an angle controller)
    //getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.LL_TURN_TOLERANCE);
        System.out.println("Align With Node - Start");
        this.swerve = swerve;
        this.limelight = limelight;
        this.vX = vX;
        this.vY = vY;
        this.pipeline = pipeline;

  }

  @Override
  public void initialize()
  {
    limelight.turnOnLED();
    limelight.setLLPipeline(pipeline);
  }

  @Override
  public boolean isFinished() {
    //never end
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
      swerve.drive(new Translation2d(0, 0), 0, false, false);
      limelight.setLLPipeline(LIMELIGHT_PIPELINE.DC);
      System.out.println("Align With Node - End");
  }

}