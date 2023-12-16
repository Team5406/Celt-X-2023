package frc.team5406.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;

public class ManualMoveExtend extends CommandBase {
    private final ExtendSubsystem extend;
  private final DoubleSupplier position;
  private final DoubleSupplier tolerance;


  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public ManualMoveExtend(ExtendSubsystem extend, DoubleSupplier position, DoubleSupplier tolerance)
  {
    this.extend = extend;
    this.position = position;
    this.tolerance = tolerance;
    addRequirements(extend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    extend.gotoExtendPosition(position.getAsDouble());
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
    extend.gotoExtendPosition(extend.getExtendAbsolutePosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
 }
}
