package frc.team5406.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class MoveWristTrapezoid extends ProfiledPIDCommand {

   private final WristSubsystem wrist;
   private final ShoulderSubsystem shoulder;
   private final double position;
 

   /*
    * Default speed
    */
  public MoveWristTrapezoid(double position, WristSubsystem wrist, ShoulderSubsystem shoulder) {
    super(
        new ProfiledPIDController(
            Constants.WRIST_PID_PROFILED_P,
            Constants.WRIST_PID_PROFILED_I,
            Constants.WRIST_PID_PROFILED_D,
            new TrapezoidProfile.Constraints(
                Constants.WRIST_MAX_SPEED,
                Constants.WRIST_MAX_ACCELERATION)),
        // Close loop on heading
        wrist::getWristAbsoluteEncoderPosition,
        // Set reference to target
        position,
        // Pipe output to turn robot
        (output, setpoint) -> wrist.useOutputPosition(output, setpoint, shoulder),
        // Require the drive
        wrist);

    // Set the controller to be continuous (because it is an angle controller)
   // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.WRIST_POSITION_TOLERANCE);

    this.position = position;
    this.wrist = wrist;
    this.shoulder = shoulder;
  }

  public MoveWristTrapezoid(double position, WristSubsystem wrist, ShoulderSubsystem shoulder, double acceleration) {
    super(
        new ProfiledPIDController(
            Constants.WRIST_PID_PROFILED_P,
            Constants.WRIST_PID_PROFILED_I,
            Constants.WRIST_PID_PROFILED_D,
            new TrapezoidProfile.Constraints(
                Constants.WRIST_MAX_SPEED,
                acceleration)),
        // Close loop on heading
        wrist::getWristAbsoluteEncoderPosition,
        // Set reference to target
        position,
        // Pipe output to turn robot
        (output, setpoint) -> wrist.useOutputPosition(output, setpoint, shoulder),
        // Require the drive
        wrist);

    // Set the controller to be continuous (because it is an angle controller)
   // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.WRIST_POSITION_TOLERANCE);

    this.position = position;
    this.wrist = wrist;
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
