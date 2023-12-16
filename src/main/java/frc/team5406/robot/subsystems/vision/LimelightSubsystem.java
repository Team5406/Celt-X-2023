package frc.team5406.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    public double robotOffset = 0;
    public String offset = "MIDDLE";
    public int currentLevel = 1;
    public void turnOffLED(){
        //turnOnLED();
        NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("ledMode").setNumber(Constants.LIMELIGHT_LED_OFF);
    }

    public void turnOnLED(){
        NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("ledMode").setNumber(Constants.LIMELIGHT_LED_ON);
    }

    public double getLLtx(){
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return tx - robotOffset;
    }

    public double getLLtv(){
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        return tv;
    }


    public double getLLta(){
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        return ta;
    }

    public double getUncorrectedLLtx(){
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        return tx - Constants.APRIL_TAG_OFFSET;
    }

    public double getLLty(){
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return ty;
    }

    public void setLLPipeline(LIMELIGHT_PIPELINE pipeline){
        int limelightPipeline;
        switch(pipeline){
            case L2:
                limelightPipeline = Constants.LIMELIGHT_PIPELINE_L2;
                currentLevel = 2;
                break;
            case L3:
                limelightPipeline = Constants.LIMELIGHT_PIPELINE_L3;
                currentLevel = 3;
                break;
            case APRILTAG:
                limelightPipeline = Constants.LIMELIGHT_PIPELINE_APRILTAG;
                break;
            case DC:
            default:
                limelightPipeline = Constants.LIMELIGHT_PIPELINE_DC;
                break;
        }
        NetworkTableInstance.getDefault().getTable(Constants.LIMELIGHT_NAME).getEntry("pipeline").setNumber(limelightPipeline);    
    }

    public void updatePos(int currentPOV){
        if(currentPOV == 180){
            offset = "LEFT";
        } else if(currentPOV == 270){
            offset ="MIDDLE";
        } else if(currentPOV == 0){
            offset ="RIGHT";
        }
    }

    
    public LimelightSubsystem(){
    }

    public void setOffset(){
        switch(offset){
            case "LEFT":
                if(currentLevel == 2){
                    robotOffset = Constants.OFFSET_ONE_L2_LIMELIGHT;
                } else {
                    robotOffset = Constants.OFFSET_ONE_L3_LIMELIGHT;
                }
                break;
            case "MIDDLE":
                if(currentLevel == 2){
                    robotOffset = Constants.OFFSET_TWO_L2_LIMELIGHT;
                } else {
                    robotOffset = Constants.OFFSET_TWO_L3_LIMELIGHT;
                }
                break;
            case "RIGHT":
                if(currentLevel == 2){
                    robotOffset = Constants.OFFSET_THREE_L2_LIMELIGHT;
                } else {
                    robotOffset = Constants.OFFSET_THREE_L3_LIMELIGHT;
                }
                break;
            case "NONE":
            default:
                robotOffset = Constants.OFFSET_TWO_L3_LIMELIGHT;
        }   

        SmartDashboard.putString("LL Current Offset", offset);
     }
    
    @Override
    public void periodic(){
        setOffset();
    }

    public enum LIMELIGHT_PIPELINE {
        L2,
        L3,
        APRILTAG,
        DC
    } 
}
