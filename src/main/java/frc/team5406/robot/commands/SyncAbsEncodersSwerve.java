package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;

public class SyncAbsEncodersSwerve extends CommandBase {

    private final SwerveSubsystem swerve;

    public SyncAbsEncodersSwerve(SwerveSubsystem swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerve.syncAbsEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true; //End Instantly
  }
}
