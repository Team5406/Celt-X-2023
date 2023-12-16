package frc.team5406.robot.commands;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem.LIMELIGHT_PIPELINE;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightDrivecam extends CommandBase{
    
    private final LimelightSubsystem limelight;

    public LimelightDrivecam(LimelightSubsystem limelight){
        this.limelight = limelight;
        addRequirements(limelight);
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
    
    @Override
    public void execute(){
        limelight.turnOffLED();
        limelight.setLLPipeline(LIMELIGHT_PIPELINE.DC);   
    }

}
