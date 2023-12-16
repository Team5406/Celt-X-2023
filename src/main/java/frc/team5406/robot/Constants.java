package frc.team5406.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.team5406.robot.lib.swervelib.math.Matter;
import frc.team5406.robot.lib.swervelib.parser.PIDFConfig;

public final class Constants {
    // General Constants
    public static final int OPERATOR_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;
    public static final double CONTROLLER_ANALOG_THRESHOLD = 0.15;

    public static final double MAXIMUM_VOLTAGE = 12.0;
    public static final double OUTPUT_RANGE_MIN = -1;
    public static final double OUTPUT_RANGE_MAX = 1;
    public static final double DEFAULT_DEADBAND = 0.05;

    public static final double SECONDS_PER_MINUTE = 60;
    public static final double DEGREES_PER_ROTATION = 360;

    public static final double MIN_PRESSURE = 90;
    public static final double MAX_PRESSURE = 105;

    public static final boolean INTAKE_EXTEND = false;
    public static final boolean INTAKE_RETRACT = true;

    public static final double ROBOT_MASS = Units.lbsToKilograms(130); // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag()

    public static final byte NAVX_FREQ = 66;
    public static final double TRAJECTORY_LOOP_TIME = 0.01;

    public static final int TRAJECTORY_TIME_INDEX = 0;
    public static final int TRAJECTORY_POSITION_INDEX = 1;
    public static final int TRAJECTORY_VELOCITY_INDEX = 2;
    public static final int TRAJECTORY_ACCELERATION_INDEX = 3;

    // Motor IDs and Inversions
    // Drive IDs are in JSON files
    public static final int SHOULDER_MOTOR_ONE = 9;
    public static final int SHOULDER_MOTOR_TWO = 10;
    public static final boolean SHOULDER_MOTOR_INVERSION = false;
    public static final boolean SHOULDER_MOTOR_FOLLOWER_INVERSION = true;

    public static final int EXTEND_MOTOR_ONE = 12;
    public static final int EXTEND_MOTOR_TWO = 11;

    public static final int WRIST_MOTOR_ONE = 13;
    public static final boolean WRIST_MOTOR_INVERSION = true;

    public static final int INTAKE_MOTOR_ONE = 14;
    public static final int INTAKE_MOTOR_TWO = 15;
    public static final boolean INTAKE_MOTOR_INVERSION = false;
    public static final boolean INTAKE_MOTOR_FOLLOWER_INVERSION = true;

    public static final int INTAKE_CYLINDER_LEFT = 0;
    public static final int INTAKE_CYLINDER_RIGHT = 1;

    // Gear Ratios
    public static final double SHOULDER_GEAR_RATIO = 20 / 1 // sport gearbox
            * 54 / 26 // spur gears
            * 48 / 16; // chain sprockets
    public static final double SHOULDER_ABSOLUTE_ENCODER_GEAR_RATIO = 36.0 / 50 * 0.925;
    public static final double EXTEND_GEAR_RATIO = 62 / 12; // spur gears
    public static final double EXTEND_ABSOLUTE_ENCODER_GEAR_RATIO = 36.0 / 60
            * 1.0 / 12;
    public static final double WRIST_GEAR_RATIO = 48 / 1; // sport gearbox
    public static final double WRIST_ABSOLUTE_ENCODER_GEAR_RATIO = 36.0 / 48;
    public static final double INTAKE_GEAR_RATIO = 4 / 1;

    // Current Limits
    public static final int SHOULDER_CURRENT_LIMIT = 40;
    public static final int SHOULDER_LOW_CURRENT_LIMIT = 20;
    public static final int EXTEND_CURRENT_LIMIT = 40;
    public static final int WRIST_CURRENT_LIMIT = 30;
    public static final int INTAKE_CURRENT_LIMIT = 20;
    public static final int INTAKE_CURRENT_LIMIT_SPIKE = 80;

    // Soft Limits
    public static final float SHOULDER_ANGLE_MAX = 69;
    public static final float SHOULDER_ANGLE_MIN = 0;

    public static final double EXTEND_PITCH_CIRCLE_DIAMETER = 1.27;
    public static final double EXTEND_PITCH_CIRCLE_CIRCUMFERENCE = Math.PI * EXTEND_PITCH_CIRCLE_DIAMETER;
    public static final double EXTEND_STROKE = 17;

    public static final double EXTEND_POSITION_MIN = -0.5;
    public static final double EXTEND_POSITION_MAX = EXTEND_STROKE + EXTEND_POSITION_MIN;

    public static final float WRIST_ANGLE_MAX = 180;
    public static final float WRIST_ANGLE_MIN = 0;

    public static final double EXTEND_ZERO_POSITION = 0; // -2;//3.5;
    public static final double EXTEND_ABSOLUTE_ENCODER_ROLLOVER = EXTEND_GEAR_RATIO
            / (EXTEND_ABSOLUTE_ENCODER_GEAR_RATIO * EXTEND_PITCH_CIRCLE_CIRCUMFERENCE);
    public static final double UNCONVERTED_ABSOLUTE_ENCODER_EXTEND = 0.6629
            - EXTEND_ZERO_POSITION / EXTEND_ABSOLUTE_ENCODER_ROLLOVER;;
    public static final double EXTEND_ABSOLUTE_ENCODER_OFFSET = EXTEND_ABSOLUTE_ENCODER_ROLLOVER
            * UNCONVERTED_ABSOLUTE_ENCODER_EXTEND;
    public static final double EXTEND_ABSOLUTE_ENCODER_ROLLOVER_THRESHOLD = 22;

    public static final double EXTEND_POSITION_CONVERSION_FACTOR = EXTEND_PITCH_CIRCLE_CIRCUMFERENCE / EXTEND_GEAR_RATIO;

    public static final double WRIST_HOME_ANGLE = -55;// 3.5;
    public static final double WRIST_ABSOLUTE_ENCODER_ROLLOVER = WRIST_ABSOLUTE_ENCODER_GEAR_RATIO
            * DEGREES_PER_ROTATION;
    public static final double UNCONVERTED_ABSOLUTE_ENCODER_WRIST = 0.807
            - WRIST_HOME_ANGLE / WRIST_ABSOLUTE_ENCODER_ROLLOVER;
    public static final double WRIST_ABSOLUTE_ENCODER_OFFSET = 0; //WRIST_ABSOLUTE_ENCODER_ROLLOVER
           // * UNCONVERTED_ABSOLUTE_ENCODER_WRIST; //FIXME: DO NOT TOUCH, WILL BREAK, PLEASE DON'T TOUCH. ASK PINTO OR CHRISTIAN IF CONFUSED.
    public static final double WRIST_ABSOLUTE_ENCODER_ROLLOVER_THRESHOLD = 245;

    public static final double SHOULDER_HOME_ANGLE2 = 2;// 3.5;
    public static final double SHOULDER_ABSOLUTE_ENCODER_ROLLOVER = SHOULDER_ABSOLUTE_ENCODER_GEAR_RATIO
            * SHOULDER_GEAR_RATIO;
    public static final double UNCONVERTED_ABSOLUTE_ENCODER_SHOULDER = 0.813
            - SHOULDER_HOME_ANGLE2 / SHOULDER_ABSOLUTE_ENCODER_ROLLOVER;
    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = SHOULDER_ABSOLUTE_ENCODER_ROLLOVER
            * UNCONVERTED_ABSOLUTE_ENCODER_SHOULDER;
    public static final double SHOULDER_ABSOLUTE_ENCODER_ROLLOVER_THRESHOLD = 74;

    // Speeds and Accelerations and Positions
    public static final double SHOULDER_MAX_SPEED = 300;
    public static final double SHOULDER_MAX_ACCELERATION = 300;

    public static final double EXTEND_MAX_SPEED = 40; // in/s
    public static final double EXTEND_MAX_ACCELERATION = 150;

    public static final double WRIST_MAX_SPEED = 400; // 170 degrees in a 1/4 s
    public static final double WRIST_MAX_ACCELERATION = 5400;
    public static final double WRIST_DEADBAND = 25; // degrees/s

    public static final double INTAKE_MAX_SPEED = 1200;

    public static final double[] WRIST_PROFILE_STOW_L3 = { 0, 53, 120, 143 };
    public static final double[] EXTEND_PROFILE_STOW_L3 = { 0, 0, 12.9, 12.9 };
    public static final double[] SHOULDER_PROFILE_STOW_L3 = { 0, 56, 56, 56 };

    public static final double[] WRIST_PROFILE_STOW_L2 = { 0, 70, 143, 143 };
    public static final double[] EXTEND_PROFILE_STOW_L2 = { 0, 0, 8, 8.1 };
    public static final double[] SHOULDER_PROFILE_STOW_L2 = { 0, 28, 51, 48 };

    public static final double[] WRIST_PROFILE_L2_L3 = { 143, 119, 130, 143 };
    public static final double[] EXTEND_PROFILE_L2_L3 = { 8.1, 11, 14, 16.8 };
    public static final double[] SHOULDER_PROFILE_L2_L3 = { 48, 58, 60, 55 };

    public static final double[] WRIST_PROFILE_L3_STOW = { 143, 91, 49, 0 };
    public static final double[] EXTEND_PROFILE_L3_STOW = { 12.9, 12.9, 0, 0 };
    public static final double[] SHOULDER_PROFILE_L3_STOW = { 56, 60, 60, 0 };

    public static final double[] WRIST_PROFILE_L2_STOW = { 143, 52, 0 };
    public static final double[] EXTEND_PROFILE_L2_STOW = { 4.6, 2.8, 0 };
    public static final double[] SHOULDER_PROFILE_L2_STOW = { 42, 44, 0 };

    public static final double[] WRIST_PROFILE_L3_L2 = { 143, 130, 119, 143 };
    public static final double[] EXTEND_PROFILE_L3_L2 = { 16.8, 14, 11, 8.1 };
    public static final double[] SHOULDER_PROFILE_L3_L2 = { 55, 60, 58, 48 };

    public static final int SHOULDER_POSITION_LEVEL_THREE = 55; // degrees
    public static final int EXTEND_LENGTH_LEVEL_THREE = 8; // inches
    public static final int WRIST_POSITION_LEVEL_THREE = 170; // degrees
    
    public static final double SHOULDER_HOME_ANGLE = 0.1;
    public static final double EXTEND_HOME_POSITION = 21.9;
    public static final double EXTEND_HOME_POSITION_DEADBAND = 0.2;
    public static final double SHOULDER_HOME_ANGLE_DEADBAND = 10;
    public static final double INTAKE_VELOCITY_DEADBAND = 20;

    // PIDs and Feedforward
    public static final int SHOULDER_PID_SLOT_SMART_MOTION = 0;
    /*
     * public static final double SHOULDER_PID0_P = 0.081612;
     * public static final double SHOULDER_PID0_I = 0;
     * public static final double SHOULDER_PID0_D = 0.078751;
     * public static final double SHOULDER_PID0_F = 0.0;
     * public static final double SHOULDER_PID0_IZ = 0;
     */

    public static final double SHOULDER_PID0_P = 2e-5;
    public static final double SHOULDER_PID0_I = 0;
    public static final double SHOULDER_PID0_D = 5e-7;
    public static final double SHOULDER_PID0_F = 0.00008;
    public static final double SHOULDER_PID0_IZ = 0;

    public static final int SHOULDER_PID_SLOT_POSITION = 1;
    public static final double SHOULDER_PID1_P =  3e-1;//0.3;
    public static final double SHOULDER_PID1_I = 0;
    public static final double SHOULDER_PID1_D = 1e-5; //0.078751;
    public static final double SHOULDER_PID1_F = 0.0;
    public static final double SHOULDER_PID1_IZ = 0;

    public static final int SHOULDER_PID_SLOT_VELOCITY = 2;
    public static final double SHOULDER_PID2_P = 1.3832E-11;
    public static final double SHOULDER_PID2_I = 0;
    public static final double SHOULDER_PID2_D = 0;
    public static final double SHOULDER_PID2_F = 0.0;
    public static final double SHOULDER_PID2_IZ = 0;

    public static final double SHOULDER_ALLOWED_ERROR = 0.05;

    public static final double SHOULDER_POSITION_TOLERANCE = 1;

    public static final double SHOULDER_PID_PROFILED_P = 12 * (2 / 0.05) / SHOULDER_MAX_SPEED; // 1 deg in 0.05s, if 12V
                                                                                               // achieves max speed;
    public static final double SHOULDER_PID_PROFILED_I = 0;
    public static final double SHOULDER_PID_PROFILED_D = 0;

    public static final double SHOULDER_KS = 0.5;
    public static final double SHOULDER_KG = 0.05; // 1.5; //0.5;
    public static final double SHOULDER_KV = 2;// 1; //0.6; //FIXME
    public static final double SHOULDER_KA = 0.0018678;

    public static final double SHOULDER_ANGLE_OFFSET = -36;

    public static final int EXTEND_PID_SLOT_SMART_MOTION = 0;
    /*
     * public static final double EXTEND_PID0_P = 1e-5;
     * public static final double EXTEND_PID0_I = 0;
     * public static final double EXTEND_PID0_D = 0.00002;
     * public static final double EXTEND_PID0_F = 0.0;
     * public static final double EXTEND_PID0_IZ = 0;
     */

    public static final double EXTEND_PID0_P = 1e-5;
    public static final double EXTEND_PID0_I = 0;
    public static final double EXTEND_PID0_D = 5e-7;
    public static final double EXTEND_PID0_F = 0.0001;
    public static final double EXTEND_PID0_IZ = 0;

    public static final int EXTEND_PID_SLOT_POSITION = 1;
    public static final double EXTEND_PID1_P = 2e-1;
    public static final double EXTEND_PID1_I = 0;
    public static final double EXTEND_PID1_D = 1e-5;
    public static final double EXTEND_PID1_F = 0.0;
    public static final double EXTEND_PID1_IZ = 0;

    public static final int EXTEND_PID_SLOT_VELOCITY = 2;
    public static final double EXTEND_PID2_P = 2.7593E-5; // -11
    public static final double EXTEND_PID2_I = 0;
    public static final double EXTEND_PID2_D = 0;
    public static final double EXTEND_PID2_F = 0.0;
    public static final double EXTEND_PID2_IZ = 0;

    public static final double EXTEND_POSITION_TOLERANCE = 0.5;
    public static final double EXTEND_ALLOWED_ERROR = 0.01;

    public static final double EXTEND_PID_PROFILED_P = 12 * (1 / 0.05) / EXTEND_MAX_SPEED; // 1 deg in 0.05s, if 12V
                                                                                           // achieves max speed;
    public static final double EXTEND_PID_PROFILED_I = 0;
    public static final double EXTEND_PID_PROFILED_D = 0;

    public static final double EXTEND_KS =0.1; //0.1;
    public static final double EXTEND_KG =0.25; //0.25; // FIXME
    public static final double EXTEND_KV =0.18;//-0.2;
    public static final double EXTEND_KA =0.0088598;

    public static final int WRIST_PID_SLOT_SMART_MOTION = 0;
    /*
     * public static final double WRIST_PID0_P = 0.036495;
     * public static final double WRIST_PID0_I = 0;
     * public static final double WRIST_PID0_D = 0.033678;
     * public static final double WRIST_PID0_F = 0.0;
     * public static final double WRIST_PID0_IZ = 0;
     */

    /*
     * public static final double WRIST_PID0_P = 0.00001;
     * public static final double WRIST_PID0_I = 0;
     * public static final double WRIST_PID0_D = 0;
     * public static final double WRIST_PID0_F = 0.0000156;
     * public static final double WRIST_PID0_IZ = 0;
     */

    public static final double WRIST_PID0_P = 1.5e-5;
    public static final double WRIST_PID0_I = 0;
    public static final double WRIST_PID0_D = 5e-7;
    public static final double WRIST_PID0_F = 0.00005;
    public static final double WRIST_PID0_IZ = 0;

    public static final int WRIST_PID_SLOT_POSITION = 1;
    public static final double WRIST_PID1_P = 1e-2;
    public static final double WRIST_PID1_I = 1e-8;
    public static final double WRIST_PID1_D = 1e-5;//0.014594;
    public static final double WRIST_PID1_F = 0.0;
    public static final double WRIST_PID1_IZ = 0;

    public static final int WRIST_PID_SLOT_VELOCITY = 2;
    public static final double WRIST_PID2_P = 0.0001; // 15;//2.6995E-23;
    public static final double WRIST_PID2_I = 0;
    public static final double WRIST_PID2_D = 0;
    public static final double WRIST_PID2_F = 0.0;
    public static final double WRIST_PID2_IZ = 0;

    public static final double WRIST_ALLOWED_ERROR = 0.001;
    public static final double WRIST_POSITION_TOLERANCE = 0.5;

    public static final double WRIST_PID_PROFILED_P = 12 * (1 / 0.05) / WRIST_MAX_SPEED; // 1 deg in 0.05s, if 12V
                                                                                         // achieves max speed;
    public static final double WRIST_PID_PROFILED_I = 0;
    public static final double WRIST_PID_PROFILED_D = 0;

    public static final double WRIST_KS = 0.4;
    public static final double WRIST_KG = -0.3;
    public static final double WRIST_KV = 0.7;
    public static final double WRIST_KA = 0;

    public static final double WRIST_ANGLE_OFFSET = -105.00;

    public static final int INTAKE_PID_SLOT_VELOCITY = 0;
    public static final double INTAKE_PID0_P = 0.00022585;
    public static final double INTAKE_PID0_I = 0;
    public static final double INTAKE_PID0_D = 0;
    public static final double INTAKE_PID0_F = 0.0;
    public static final double INTAKE_PID0_IZ = 0;

    public static final double INTAKE_KS = 0.065831;
    public static final double INTAKE_KV = 0.46157;
    public static final double INTAKE_KA = 0.056747;

    public static final double SHOULDER_HOLDING_VOLTAGE = -1;
    public static final double EXTEND_HOLDING_VOLTAGE = -0.6;

    public static final int WRIST_OUTTAKE_ANGLE = 130; // degrees
    public static final int WRIST_INTAKE_ANGLE = 130; // degrees

    public static final double WRIST_STOWED_ANGLE = 1;

    public static final int WRIST_SCORING_ANGLE = 170;
    public static final int EXTEND_SCORING_POSITION = 8;
    public static final int SHOULDER_SCORING_ANGLE = 55;
    public static final double INTAKE_SPEED_CONE = -350;
    public static final double INTAKE_SPEED_CUBE = 350;
    public static final double OUTTAKE_SPEED_CONE = 1000;
    public static final double OUTTAKE_SPEED_CUBE = -15;
    public static final double INTAKE_CURRENT_SPIKE_CONE = 1;
    public static final double INTAKE_VOLTAGE_HOLDING_CONE = -1;
    public static final double INTAKE_CURRENT_SPIKE_CUBE = 1;
    public static final double INTAKE_VOLTAGE_HOLDING_CUBE = 0.75;
    public static final int INTAKE_CURRENT_SPIKE_MAX_ITERATOR = 11; // 220 ms
    public static final double THROW_SPEED_CONE = 1000;
    public static final double THROW_SPEED_CUBE = -1500;

    public static final double WRIST_ENCODER_RESET_MAX_VELOCITY = WRIST_MAX_SPEED * 0.01; // 1% of max speed
    public static final double SHOULDER_ENCODER_RESET_MAX_VELOCITY = SHOULDER_MAX_SPEED * 0.01; // 1% of max speed
    public static final double EXTEND_ENCODER_RESET_MAX_VELOCITY = EXTEND_MAX_SPEED * 0.01; // 1% of max speed
    public static final int ENCODER_RESET_ITERATIONS = 500;
    public static final int ENCODER_RESET_ITERATIONS_CHANGE_STATE = 5;

    public static final String APRIL_TAG_CAMERA_NAME = "aprilTagCamera";
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(0.073, 0.252, 0.964),
            new Rotation3d(0, 0, 0));
    public static final double WRIST_HOME_POSITION = 7;
    public static final double WRIST_CONE_POSITION = 142;
    public static final double WRIST_CUBE_POSITION = 135;

    public static final double EXTEND_L2_POSITION = 6;
    public static final double EXTEND_L4_POSITION = 15;

    public static final double MAX_SPEED_METERS_PER_SECOND = 2.5;
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 0.5;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PIDFConfig xAutoPID = new PIDFConfig(5, 0, 0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(5, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(1, 0, 0.005);

    public static final double MAX_SPEED = 4;
    public static final double MAX_ACCELERATION = 2;

    public static final double LL_TURN_P = 0.15;
    public static final double LL_TURN_I = 0;
    public static final double LL_TURN_D = 1e-5;
    public static final double LL_TURN_TOLERANCE = 0.2; //degrees 
    public static final double LL_TURN_RATE_TOLERANCE = 5; //degrees per second

    public static final double ALIGN_WITH_DIRECTION_ROTATION_TOLERANCE = 0.2; //degrees

    public static final double DRIVE_CLIMB_P = 0;
    public static final double DRIVE_CLIMB_I = 0;
    public static final double DRIVE_CLIMB_D = 0;

    public static final int DRIVE_CLIMB_RISING_THRESHOLD = 16;
    public static final int DRIVE_CROSS_RISING_THRESHOLD = -10;
    public static final int DRIVE_CLIMB_LEVEL_THRESHOLD = 10;
    public static final int DRIVE_CLIMB_FALLING_THRESHOLD = 15;
    public static final int DRIVE_CROSS_FALLING_THRESHOLD = -5;
    public static final double DRIVE_CLIMB_RISING_SPEED = 1.1;
    public static final double DRIVE_CLIMB_FAST_SPEED = 1.7;
    public static final double DRIVE_CLIMB_LEVEL_SPEED_MULTIPLIER = 0.02;

    public static final int LIMELIGHT_PIPELINE_DC = 0;
    public static final int LIMELIGHT_PIPELINE_L2 = 1;
    public static final int LIMELIGHT_PIPELINE_L3 = 2;

    public static final String LIMELIGHT_NAME = "limelight";

    public static final double WRIST_EXTEND_FAST_ACCELERATION = 9000;
    public static final double WRIST_EXTEND_SLOW_ACCELERATION = 600;
    public static final double WRIST_SHOOT_POSITION = 20;
    public static final double WRIST_OUTAKE_POSITION = 95;
    public static final double WRIST_SUBSTATION_CUBE_POSITION = 0;
    public static final double WRIST_SUBSTATION_CONE_POSITION = 72.3;
    public static final int LIMELIGHT_LED_OFF = 1;
    public static final int LIMELIGHT_LED_ON = 3;

    //ONE = LEFT; TWO = MIDDLE; THREE = RIGHT
    public static final double OFFSET_ONE_L2_LIMELIGHT = -3.75;
    public static final double OFFSET_TWO_L2_LIMELIGHT = 1.3;
    public static final double OFFSET_THREE_L2_LIMELIGHT = 4.05;


    public static final double OFFSET_ONE_L3_LIMELIGHT = -3.2;
    public static final double OFFSET_TWO_L3_LIMELIGHT = 1.21;
    public static final double OFFSET_THREE_L3_LIMELIGHT = 4.3;

    public static final int LOOP_ITERATOR = 5;
    public static final boolean EXTEND_MOTOR_INVERSION = false;
    public static final double CAN_RETRY_DELAY = 0.01;
    public static final int LIMELIGHT_PIPELINE_APRILTAG = 3;
    public static final boolean WRIST_ABS_ENCODER_INVERSION = true;
    public static final double APRIL_TAG_OFFSET = -2.0;

}
