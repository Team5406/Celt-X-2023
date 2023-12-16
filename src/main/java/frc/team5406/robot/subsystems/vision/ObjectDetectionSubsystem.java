package frc.team5406.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetectionSubsystem extends SubsystemBase {
    public boolean llHasValidTarget = false;

    public double foundCone = 0;
    public double foundCube = 0;

    public double coneDistance = 0;
    public double cubeDistance = 0;
    double frameCenter = 410;

    public void updateOrangepiTracking() {
        
        foundCone = Double.valueOf(NetworkTableInstance.getDefault().getTable("orangepi").getEntry("foundCone").getString("0"));
        foundCube = Double.valueOf(NetworkTableInstance.getDefault().getTable("orangepi").getEntry("foundCube").getString("0"));
            
        if (foundCone == 1) {
            double coneCenter = Double.valueOf(NetworkTableInstance.getDefault().getTable("orangepi").getEntry("coneCenter").getString("0"));
            coneDistance = coneCenter-frameCenter;
        }else{
            coneDistance = 0;
        }
        if (foundCube == 1){
            double cubeCenter = Double.valueOf(NetworkTableInstance.getDefault().getTable("orangepi").getEntry("cubeCenter").getString("0"));
            cubeDistance = cubeCenter-frameCenter;
        }else{
            cubeDistance = 0;
        }
    }

    public double getCenterDistance(boolean cone){
        double centerDistance = 0;
        if(cone){
            centerDistance = coneDistance;
        }else{
            centerDistance = cubeDistance;
        }
        SmartDashboard.putNumber("oPi Center Distance", centerDistance);
        return centerDistance;
    }

    @Override
    public void periodic() {
        updateOrangepiTracking();
    }

}