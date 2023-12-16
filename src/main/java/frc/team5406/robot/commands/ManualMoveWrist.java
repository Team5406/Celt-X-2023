package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;

public class ManualMoveWrist extends CommandBase {
    private final WristSubsystem wrist;
  private final DoubleSupplier angle;
  private final DoubleSupplier tolerance;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public ManualMoveWrist(WristSubsystem wrist, DoubleSupplier angle, DoubleSupplier tolerance)
  {
    this.wrist = wrist;
    this.angle = angle;
    this.tolerance = tolerance;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    wrist.gotoWristAngle(angle.getAsDouble());
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
    wrist.gotoWristAngle(wrist.getWristAbsoluteEncoderPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true; //Always end
 }
}
