package frc.team5406.robot.subsystems.superstructure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ArmProfileSubsystem.ArmStates;


public class ExtendSubsystem extends SubsystemBase {


  private CANSparkMax extendMotor = new CANSparkMax(Constants.EXTEND_MOTOR_ONE, MotorType.kBrushless);
  private CANSparkMax extendMotorFollower = new CANSparkMax(Constants.EXTEND_MOTOR_TWO, MotorType.kBrushless);
  private RelativeEncoder extendEncoder;
  private AbsoluteEncoder extendAbsoluteEncoder;
  private SparkMaxPIDController extendPID;
  ArmFeedforward extendFF = new ArmFeedforward(Constants.EXTEND_KS, Constants.EXTEND_KG, Constants.EXTEND_KV);
  ArmFeedforward extendFFWithKA = new ArmFeedforward(Constants.EXTEND_KS, Constants.EXTEND_KG, Constants.EXTEND_KV, Constants.EXTEND_KA);
  private ProfiledPIDController extendPIDController = new ProfiledPIDController(Constants.EXTEND_PID0_P, Constants.EXTEND_PID0_I, Constants.EXTEND_PID0_D, 
  new Constraints(Constants.EXTEND_MAX_SPEED, Constants.EXTEND_MAX_ACCELERATION));

  private double extendPosition = 0;
  private int extendSyncCounter;
  public boolean initGood = true;
  public boolean success = false;

  private ArmStates lastArmState = ArmStates.STOW;


  public void setupMotors(){
    
    extendEncoder = extendMotor.getEncoder();
    extendAbsoluteEncoder = extendMotor.getAbsoluteEncoder(Type.kDutyCycle);
    extendPID = extendMotor.getPIDController();

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotor.restoreFactoryDefaults() == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotorFollower.restoreFactoryDefaults() == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotor.setCANTimeout(50) == REVLibError.kOk){
      success = true;
      break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      if (extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10) != REVLibError.kOk){
        errorCount++;
      }
      if (extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50) != REVLibError.kOk){
        errorCount++;
      }
      if (extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50) != REVLibError.kOk){
        errorCount++;
      }
      if (extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200) != REVLibError.kOk){
        errorCount++;
      }
      if (extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000) != REVLibError.kOk){
        errorCount++;
      }
      if (extendMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50) != REVLibError.kOk){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

    if(errorCount == 0){
      success = true;
      break;
      }
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      if (extendMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100) != REVLibError.kOk){
        errorCount++;
      }
      if (extendMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250) != REVLibError.kOk){
        errorCount++;
      }
      if (extendMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250) != REVLibError.kOk){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

    if(errorCount == 0){
      success = true;
      break;
      }
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotorFollower.follow(extendMotor, true) == REVLibError.kOk){
      success = true;
      break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendMotor.setInverted(Constants.EXTEND_MOTOR_INVERSION);
      if (extendMotor.getInverted() == Constants.EXTEND_MOTOR_INVERSION){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendMotor.setIdleMode(IdleMode.kCoast);
      if (extendMotor.getIdleMode() == IdleMode.kCoast){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendMotorFollower.setIdleMode(IdleMode.kCoast);
      if (extendMotorFollower.getIdleMode() == IdleMode.kCoast){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendPID.setFeedbackDevice(extendAbsoluteEncoder) == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotor.enableSoftLimit(SoftLimitDirection.kForward, false) == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, false) == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotorFollower.setSmartCurrentLimit(Constants.EXTEND_CURRENT_LIMIT) == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotor.setSmartCurrentLimit(Constants.EXTEND_CURRENT_LIMIT) == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
    
    success = false;
    for(int i = 0; i < 5; i++){
      extendEncoder.setPositionConversionFactor(Constants.EXTEND_PITCH_CIRCLE_CIRCUMFERENCE / Constants.EXTEND_GEAR_RATIO);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if (!checkFailedVal(extendEncoder.getPositionConversionFactor(), (Constants.EXTEND_PITCH_CIRCLE_CIRCUMFERENCE / Constants.EXTEND_GEAR_RATIO))){
      success = true;
      break;
      }
      System.out.println("Failed to set encoder pos factor" + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendAbsoluteEncoder.setPositionConversionFactor(Constants.EXTEND_GEAR_RATIO / (Constants.EXTEND_ABSOLUTE_ENCODER_GEAR_RATIO*Constants.EXTEND_PITCH_CIRCLE_CIRCUMFERENCE));
      if (!checkFailedVal(extendAbsoluteEncoder.getPositionConversionFactor(), Constants.EXTEND_GEAR_RATIO / (Constants.EXTEND_ABSOLUTE_ENCODER_GEAR_RATIO*Constants.EXTEND_PITCH_CIRCLE_CIRCUMFERENCE))){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendEncoder.setVelocityConversionFactor(Constants.EXTEND_PITCH_CIRCLE_CIRCUMFERENCE / (Constants.EXTEND_GEAR_RATIO * Constants.SECONDS_PER_MINUTE));
      if (!checkFailedVal(extendEncoder.getVelocityConversionFactor(), Constants.EXTEND_PITCH_CIRCLE_CIRCUMFERENCE / (Constants.EXTEND_GEAR_RATIO * Constants.SECONDS_PER_MINUTE))){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendAbsoluteEncoder.setInverted(true);
      if (extendAbsoluteEncoder.getInverted()){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendAbsoluteEncoder.setZeroOffset(Constants.EXTEND_ABSOLUTE_ENCODER_OFFSET);
      if (!checkFailedVal(extendAbsoluteEncoder.getZeroOffset(), Constants.EXTEND_ABSOLUTE_ENCODER_OFFSET)){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
    
    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      extendPID.setP(Constants.EXTEND_PID0_P, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getP(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_PID0_P)){
        errorCount++;
      }
      extendPID.setI(Constants.EXTEND_PID0_I, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getI(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_PID0_I)){
        errorCount++;
      }
      extendPID.setD(Constants.EXTEND_PID0_D, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getD(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_PID0_D)){
        errorCount++;
      }
      extendPID.setIZone(Constants.EXTEND_PID0_IZ, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getIZone(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_PID0_IZ)){
        errorCount++;
      }
      extendPID.setFF(Constants.EXTEND_PID0_F, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getFF(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_PID0_F)){
        errorCount++;
      }
      extendPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getOutputMax(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if (checkFailedVal(extendPID.getOutputMin(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

    if(errorCount == 0){
      success = true;
      break;
      }
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      extendPID.setP(Constants.EXTEND_PID1_P, Constants.EXTEND_PID_SLOT_POSITION);
      if (checkFailedVal(extendPID.getP(Constants.EXTEND_PID_SLOT_POSITION), Constants.EXTEND_PID1_P)){
        errorCount++;
      }
      extendPID.setI(Constants.EXTEND_PID1_I, Constants.EXTEND_PID_SLOT_POSITION);
      if (checkFailedVal(extendPID.getI(Constants.EXTEND_PID_SLOT_POSITION), Constants.EXTEND_PID1_I)){
        errorCount++;
      }
      extendPID.setD(Constants.EXTEND_PID1_D, Constants.EXTEND_PID_SLOT_POSITION);
      if (checkFailedVal(extendPID.getD(Constants.EXTEND_PID_SLOT_POSITION), Constants.EXTEND_PID1_D)){
        errorCount++;
      }
      extendPID.setIZone(Constants.EXTEND_PID1_IZ, Constants.EXTEND_PID_SLOT_POSITION);
      if (checkFailedVal(extendPID.getIZone(Constants.EXTEND_PID_SLOT_POSITION), Constants.EXTEND_PID1_IZ)){
        errorCount++;
      }
      extendPID.setFF(Constants.EXTEND_PID1_F, Constants.EXTEND_PID_SLOT_POSITION);
      if (checkFailedVal(extendPID.getFF(Constants.EXTEND_PID_SLOT_POSITION), Constants.EXTEND_PID1_F)){
        errorCount++;
      }
      extendPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.EXTEND_PID_SLOT_POSITION);
      if (checkFailedVal(extendPID.getOutputMax(Constants.EXTEND_PID_SLOT_POSITION), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if (checkFailedVal(extendPID.getOutputMin(Constants.EXTEND_PID_SLOT_POSITION), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

    if(errorCount == 0){
      success = true;
      break;
      }
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      extendPID.setP(Constants.EXTEND_PID2_P, Constants.EXTEND_PID_SLOT_VELOCITY);
      if (checkFailedVal(extendPID.getP(Constants.EXTEND_PID_SLOT_VELOCITY), Constants.EXTEND_PID2_P)){
        errorCount++;
      }
      extendPID.setI(Constants.EXTEND_PID2_I, Constants.EXTEND_PID_SLOT_VELOCITY);
      if (checkFailedVal(extendPID.getI(Constants.EXTEND_PID_SLOT_VELOCITY), Constants.EXTEND_PID2_I)){
        errorCount++;
      }
      extendPID.setD(Constants.EXTEND_PID2_D, Constants.EXTEND_PID_SLOT_VELOCITY);
      if (checkFailedVal(extendPID.getD(Constants.EXTEND_PID_SLOT_VELOCITY), Constants.EXTEND_PID2_D)){
        errorCount++;
      }
      extendPID.setIZone(Constants.EXTEND_PID2_IZ, Constants.EXTEND_PID_SLOT_VELOCITY);
      if (checkFailedVal(extendPID.getIZone(Constants.EXTEND_PID_SLOT_VELOCITY), Constants.EXTEND_PID2_IZ)){
        errorCount++;
      }
      extendPID.setFF(Constants.EXTEND_PID2_F, Constants.EXTEND_PID_SLOT_VELOCITY);
      if (checkFailedVal(extendPID.getFF(Constants.EXTEND_PID_SLOT_VELOCITY), Constants.EXTEND_PID2_F)){
        errorCount++;
      }
      extendPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.EXTEND_PID_SLOT_VELOCITY);
      if (checkFailedVal(extendPID.getOutputMax(Constants.EXTEND_PID_SLOT_VELOCITY), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if (checkFailedVal(extendPID.getOutputMin(Constants.EXTEND_PID_SLOT_VELOCITY), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

    if(errorCount == 0){
      success = true;
      break;
      }
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      extendPID.setSmartMotionMaxVelocity(Constants.EXTEND_MAX_SPEED, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getSmartMotionMaxVelocity(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_MAX_SPEED)){
        errorCount++;
      }
      extendPID.setSmartMotionMaxAccel(Constants.EXTEND_MAX_ACCELERATION, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getSmartMotionMaxAccel(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_MAX_ACCELERATION)){
        errorCount++;
      }
      extendPID.setSmartMotionAllowedClosedLoopError(Constants.EXTEND_ALLOWED_ERROR, Constants.EXTEND_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(extendPID.getSmartMotionAllowedClosedLoopError(Constants.EXTEND_PID_SLOT_SMART_MOTION), Constants.EXTEND_ALLOWED_ERROR)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

    if(errorCount == 0){
      success = true;
      break;
      }
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      extendPIDController.setTolerance(Constants.EXTEND_POSITION_TOLERANCE);
      if (!checkFailedVal(extendPIDController.getPositionTolerance(), Constants.EXTEND_POSITION_TOLERANCE)){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendEncoder.setPosition(0.5) == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotor.burnFlash() == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if (extendMotorFollower.burnFlash() == REVLibError.kOk){
      success = true;
      break;
      }
    Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success){
      System.out.println("Error at line Extend: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
    
  stopExtend(); 
  SmartDashboard.putBoolean("Extend Initialize", initGood);   
  }

  public double getExtendPosition() {
    return extendEncoder.getPosition();
  }

  public double getExtendVelocity() {
    return extendEncoder.getVelocity();
  }

  public double getExtendAbsolutePosition() {
    double position = extendAbsoluteEncoder.getPosition();
      return position;
  }

  public void disableExtendSoftLimits(){
    disableExtendForwardSoftLimit();
    disableExtendReverseSoftLimit();
  }

  public void disableExtendReverseSoftLimit(){
    extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void disableExtendForwardSoftLimit(){
    extendMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
  }

  public void enableExtendSoftLimits(){
    enableExtendForwardSoftLimit();
    enableExtendReverseSoftLimit();
  }

  public void enableExtendForwardSoftLimit(){
    extendMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  public void enableExtendReverseSoftLimit(){
    extendMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }
  
  public void resetExtendPosition() {
    //extendEncoder.setPosition(0);
    extendEncoder.setPosition(getExtendAbsolutePosition());
  }


  public void stopExtend() {
      gotoExtendPosition(Constants.EXTEND_HOME_POSITION);

    }

  public void syncExtendAbsAngle(ArmProfileSubsystem armProfile){
      if(Math.abs(getExtendVelocity())< Constants.EXTEND_ENCODER_RESET_MAX_VELOCITY && armProfile.getArmState() == ArmStates.STOW){
        extendSyncCounter++;
        if((extendSyncCounter >= Constants.ENCODER_RESET_ITERATIONS) || (lastArmState != armProfile.getArmState() && extendSyncCounter >= Constants.ENCODER_RESET_ITERATIONS_CHANGE_STATE)){
          lastArmState = armProfile.getArmState();
          extendSyncCounter = 0;
          extendEncoder.setPosition(getExtendAbsolutePosition());
        } 
      } else {
        extendSyncCounter = 0;
      }
    }
  
  public void setExtendSpeed(double speed) {
    extendMotor.set(speed);
    SmartDashboard.putNumber("Extend Input", speed);

    extendPosition = getExtendPosition();

  }

  public void useOutputVoltage(double output, TrapezoidProfile.State setpoint, ShoulderSubsystem shoulder) {
    // Calculate the feedforward from the sepoint
    double shoulderAngle = shoulder.getShoulderAngle();
    //double feedforward = extendFF.calculate(setpoint.position, setpoint.velocity);
    double arbFF = extendFF.calculate(Units.degreesToRadians(shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 90), Units.degreesToRadians(setpoint.velocity)); 
    // Add the feedforward to the PID output to get the motor output
    extendMotor.setVoltage(output + arbFF);
  }

  public void useOutputPosition(double output, TrapezoidProfile.State setpoint, ShoulderSubsystem shoulder) {
    double shoulderAngle = shoulder.getShoulderAbsoluteEncoderPosition();
    double angle = shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET;

    SmartDashboard.putNumber("Wrist Abs Angle", angle);
    
    //double feedforward = extendFF.calculate(setpoint.position, setpoint.velocity);
    double arbFF = extendFF.calculate(Units.degreesToRadians(angle), (setpoint.velocity)); 

    // Add the feedforward to the PID output to get the motor output
    extendPID.setReference(setpoint.position, ControlType.kPosition, Constants.EXTEND_PID_SLOT_POSITION, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public void setExtendVoltage(double voltage){
    extendMotor.setVoltage(voltage);
    System.out.println("extend set");
  }

public void gotoExtendPosition(double position) {
  extendPID.setReference(position, ControlType.kPosition, Constants.EXTEND_PID_SLOT_POSITION);  
}

  public ExtendSubsystem(){
    setupMotors();
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Extend Pos", getExtendPosition());
    SmartDashboard.putNumber("Extend Absolute Pos", getExtendAbsolutePosition());
    SmartDashboard.putNumber("Extend Vel", getExtendVelocity());
    SmartDashboard.putNumber("Current Extend", extendMotor.getOutputCurrent());

  }

  public void setArmLevelThree(){
    lastArmState = ArmStates.L3;
  }

  public void setArmLevelTwo(){
    lastArmState = ArmStates.L2;
  }

  public ArmStates getArmLevel(){
    return lastArmState;
  }

  private boolean checkFailedVal(double x, double y){
    double z = x - y;
    if(z == 0){
      return false;
    } else {
      double decimalPlaces = (int)-Math.log10(x);
      return !(Math.floor(Math.abs(z * Math.pow(10, decimalPlaces + 1))) == 0);
    }
  }
    
}
