package frc.team5406.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class MoveShoulderTrapezoid extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

   private final ExtendSubsystem extend;
   private final ShoulderSubsystem shoulder;
   private final double position;
 
  public MoveShoulderTrapezoid(double position, ExtendSubsystem extend, ShoulderSubsystem shoulder) {
    super(
        new ProfiledPIDController(
            Constants.SHOULDER_PID_PROFILED_P,
            Constants.SHOULDER_PID_PROFILED_I,
            Constants.SHOULDER_PID_PROFILED_D,
            new TrapezoidProfile.Constraints(
                Constants.SHOULDER_MAX_SPEED,
                Constants.SHOULDER_MAX_ACCELERATION)),
        // Close loop on heading
        shoulder::getShoulderAbsoluteEncoderPosition,
        // Set reference to target
        position,
        // Pipe output to turn robot
        (output, setpoint) -> shoulder.useOutputPosition(output, setpoint, extend),
        // Require the drive
        shoulder);

    // Set the controller to be continuous (because it is an angle controller)
   // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.SHOULDER_POSITION_TOLERANCE);

    this.position = position;
    this.extend = extend;
    this.shoulder = shoulder;
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }

  @Override
  public void end(boolean interrupted){
    
  }
}
