package frc.team5406.robot.subsystems.superstructure;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class ArmProfileSubsystem {
     public enum Mechanisms {
          SHOULDER, 
          EXTEND, 
          WRIST
     };
     public enum Profiles {
          STOW_L3(ArmStates.L3, "STOW_L3"),
          L3_STOW(ArmStates.STOW, "L3_STOW"),
          L3_SCORE(ArmStates.STOW, "L3_SCORE"),
          L2_SCORE(ArmStates.STOW, "L2_SCORE"),
          STOW_L2(ArmStates.L2, "STOW_L2"),
          L2_STOW(ArmStates.STOW, "L2_STOW"),
          L2_L3(ArmStates.L3, "L2_L3"),
          L3_L2(ArmStates.L2, "L3_L2"),
          STOW_CONEINTAKE(ArmStates.CONEINTAKE, "STOW_CONEINTAKE"),
          STOW_CUBEINTAKE(ArmStates.CUBEINTAKE, "STOW_CUBEINTAKE"),
          CUBEINTAKE_STOW(ArmStates.STOW, "CUBEINTAKE_STOW"),
          CONEINTAKE_STOW(ArmStates.STOW, "CONEINTAKE_STOW"),
          CUBEINTAKE_CONEINTAKE(ArmStates.CONEINTAKE, "CUBEINTAKE_CONEINTAKE"),
          CONEINTAKE_CUBEINTAKE(ArmStates.CUBEINTAKE, "CONEINTAKE_CUBEINTAKE");
          public String name;
          public ArmStates endState;
          Profiles(ArmStates state, String name){
               this.endState = state;
               this.name = name;
          }
     };

     public enum ArmStates {
          STOW,
          L2,
          L3,
          CONEINTAKE,
          CUBEINTAKE
     };
     
     public class Profile {
          public int length;
          public EnumMap<Mechanisms, List<double[]>> paths = new EnumMap<>(Mechanisms.class);
     }

     public EnumMap<Profiles, Profile> trajectories = new EnumMap<>(Profiles.class);
     private ArmStates armState = ArmStates.STOW;

     public ArmProfileSubsystem(){
          for (Profiles path : Profiles.values()) {
               Profile profile = new Profile();
               for (Mechanisms mechanism : Mechanisms.values()) {

                    String filename = mechanism + "_" + path;
                    try{
                         List<double[]> trajectory;
                         trajectory =           
                         Files.lines(Paths.get(Filesystem.getDeployDirectory() + "/armprofile/"+filename+".csv"))
                         .skip(1) // Skip the heading
                         .map(line -> Arrays.stream(line.split(","))
                                        .mapToDouble(Double::parseDouble)
                                        .toArray()
                              )
                         .collect(Collectors.toList());
                         profile.paths.put(mechanism, trajectory);
                         profile.length = trajectory.size();
                         //printData(trajectory);
                    } catch(IOException e){
                         DriverStation.reportError("Unable to parse the armprofile " +  filename +" please restart the robot.", null);
                    }
               }
               trajectories.put(path, profile);
          }
     }

     public void setArmState(ArmStates state){
          armState = state;
     }

     public ArmStates getArmState(){
          return armState;
     }

     public void printData(List<double[]> trajectory){
          for(int i = 0; i<trajectory.size(); i++){
               double[] array = trajectory.get(i);
               for(int x = 0; x < array.length; x++){
                    //System.out.println(array[0] + ", " + array[1] + ", " + array[2] + ", " + array[3]);
                    System.out.print(array[x] + ", ");
               }
               System.out.println();
          }
     }
     }

