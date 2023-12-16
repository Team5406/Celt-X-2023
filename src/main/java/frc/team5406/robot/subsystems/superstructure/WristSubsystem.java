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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.superstructure.ArmProfileSubsystem.ArmStates;


public class WristSubsystem extends SubsystemBase {

  private CANSparkMax wristMotor = new CANSparkMax(Constants.WRIST_MOTOR_ONE, MotorType.kBrushless);

  private RelativeEncoder wristEncoder;
  private AbsoluteEncoder wristAbsoluteEncoder;
  private SparkMaxPIDController wristPID;

  ArmFeedforward wristFF = new ArmFeedforward(Constants.WRIST_KS, Constants.WRIST_KG, Constants.WRIST_KV, Constants.WRIST_KA);
  SimpleMotorFeedforward wristFF2 = new SimpleMotorFeedforward(Constants.WRIST_KS, Constants.WRIST_KV);
  private ProfiledPIDController wristPIDController = new ProfiledPIDController(Constants.WRIST_PID_PROFILED_P, Constants.WRIST_PID_PROFILED_I, Constants.WRIST_PID_PROFILED_D, 
  new Constraints(Constants.WRIST_MAX_SPEED, Constants.WRIST_MAX_ACCELERATION));

  public boolean wristReset = false;
  private double wristAngle = 0;

  private int wristSyncCounter = 0;
  private ArmStates lastArmState = ArmStates.STOW;
  public boolean initGood = true;
  public boolean success = false;

  public void setupMotors(){

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      if(wristMotor.setCANTimeout(50) != REVLibError.kOk){
        errorCount++;
      }
      if(wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10) != REVLibError.kOk){
        errorCount++;
      }
      if(wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50) != REVLibError.kOk){
        errorCount++;
      }
      if(wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50) != REVLibError.kOk){
        errorCount++;
      }
      if(wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200) != REVLibError.kOk){
        errorCount++;
      }
      if(wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000) != REVLibError.kOk){
        errorCount++;
      }
      if(wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50) != REVLibError.kOk){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    
    if(errorCount == 0){
      success = true;
      break;
    }
   }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
  
    success = false;
    for(int i = 0; i < 5; i++){
      if(wristMotor.restoreFactoryDefaults() == REVLibError.kOk){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      wristMotor.setInverted(Constants.WRIST_MOTOR_INVERSION);
      if(wristMotor.getInverted() == Constants.WRIST_MOTOR_INVERSION){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      wristMotor.setIdleMode(IdleMode.kCoast);
      if(wristMotor.getIdleMode() == IdleMode.kCoast){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    wristEncoder = wristMotor.getEncoder();
    wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    wristPID = wristMotor.getPIDController();

    success = false;
    for(int i = 0; i < 5; i++){
      if(wristPID.setFeedbackDevice(wristAbsoluteEncoder) == REVLibError.kOk){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if(wristMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.WRIST_ANGLE_MAX) == REVLibError.kOk 
      && wristMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.WRIST_ANGLE_MIN) == REVLibError.kOk){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
    
    success = false;
    for(int i = 0; i < 5; i++){
      if(wristMotor.enableSoftLimit(SoftLimitDirection.kForward, false) == REVLibError.kOk 
      && wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, false) == REVLibError.kOk){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      if(wristMotor.setSmartCurrentLimit(Constants.WRIST_CURRENT_LIMIT) == REVLibError.kOk){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
    
    //disableWristSoftLimits();
    
    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      wristEncoder.setPositionConversionFactor(Constants.DEGREES_PER_ROTATION / Constants.WRIST_GEAR_RATIO);
    if(checkFailedVal(wristEncoder.getPositionConversionFactor(), Constants.DEGREES_PER_ROTATION / Constants.WRIST_GEAR_RATIO)){
      errorCount++;
    }
    wristEncoder.setVelocityConversionFactor(Constants.DEGREES_PER_ROTATION / (Constants.WRIST_GEAR_RATIO * Constants.SECONDS_PER_MINUTE));
    if(checkFailedVal(wristEncoder.getVelocityConversionFactor(), Constants.DEGREES_PER_ROTATION / (Constants.WRIST_GEAR_RATIO * Constants.SECONDS_PER_MINUTE))){
      errorCount++;
    }
    if(errorCount == 0){
      success = true;
      break;
    }
    Timer.delay(Constants.CAN_RETRY_DELAY);
      }
  if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      wristAbsoluteEncoder.setInverted(Constants.WRIST_ABS_ENCODER_INVERSION);
      if(wristAbsoluteEncoder.getInverted() == Constants.WRIST_ABS_ENCODER_INVERSION){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }


   // wristAbsoluteEncoder.setPositionConversionFactor(1);
    //wristAbsoluteEncoder.setZeroOffset(0);
    
    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      wristAbsoluteEncoder.setPositionConversionFactor(Constants.WRIST_ABSOLUTE_ENCODER_GEAR_RATIO*Constants.DEGREES_PER_ROTATION);
      if(checkFailedVal(wristAbsoluteEncoder.getPositionConversionFactor(), Constants.WRIST_ABSOLUTE_ENCODER_GEAR_RATIO*Constants.DEGREES_PER_ROTATION)){
        errorCount++;
      }
      wristAbsoluteEncoder.setVelocityConversionFactor(Constants.WRIST_ABSOLUTE_ENCODER_GEAR_RATIO*Constants.DEGREES_PER_ROTATION);
      if(checkFailedVal(wristAbsoluteEncoder.getVelocityConversionFactor(), Constants.WRIST_ABSOLUTE_ENCODER_GEAR_RATIO*Constants.DEGREES_PER_ROTATION)){
        errorCount++;
      }
      if(errorCount == 0){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      wristAbsoluteEncoder.setZeroOffset(Constants.WRIST_ABSOLUTE_ENCODER_OFFSET);
      if(checkFailedVal(wristAbsoluteEncoder.getZeroOffset(), Constants.WRIST_ABSOLUTE_ENCODER_OFFSET)){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
   
    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      wristPID.setP(Constants.WRIST_PID0_P, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getP(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_PID0_P)){
        errorCount++;
      }
      wristPID.setI(Constants.WRIST_PID0_I, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getI(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_PID0_I)){
        errorCount++;
      }
      wristPID.setD(Constants.WRIST_PID0_D, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getD(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_PID0_D)){
        errorCount++;
      }
      wristPID.setIZone(Constants.WRIST_PID0_IZ, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getIZone(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_PID0_IZ)){
        errorCount++;
      }
      wristPID.setFF(Constants.WRIST_PID0_F, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getFF(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_PID0_F)){
        errorCount++;
      }

      wristPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getOutputMax(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if(checkFailedVal(wristPID.getOutputMin(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    
    if(errorCount == 0){
      success = true;
      break;
    }
   }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      wristPID.setPositionPIDWrappingEnabled(true);
      if(!wristPID.getPositionPIDWrappingEnabled()){
        errorCount++;
      }
      wristPID.setPositionPIDWrappingMinInput(0);
      if(checkFailedVal(wristPID.getPositionPIDWrappingMaxInput(), 0)){
        errorCount++;
      }
      wristPID.setPositionPIDWrappingMaxInput(360);
      if(checkFailedVal(wristPID.getPositionPIDWrappingMaxInput(), 360)){
        errorCount++;
      }
      if(errorCount == 0){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
    
    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      wristPID.setSmartMotionMaxVelocity(Constants.WRIST_MAX_SPEED, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getSmartMotionMaxVelocity(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_MAX_SPEED)){
        errorCount++;
      }
      wristPID.setSmartMotionMaxAccel(Constants.WRIST_MAX_ACCELERATION, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getSmartMotionMaxAccel(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_MAX_ACCELERATION)){
        errorCount++;
      }
      wristPID.setSmartMotionAllowedClosedLoopError(Constants.WRIST_ALLOWED_ERROR, Constants.WRIST_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(wristPID.getSmartMotionAllowedClosedLoopError(Constants.WRIST_PID_SLOT_SMART_MOTION), Constants.WRIST_ALLOWED_ERROR)){
        errorCount++;
      }
      if(errorCount == 0){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      wristPID.setP(Constants.WRIST_PID1_P, Constants.WRIST_PID_SLOT_POSITION);
      if(checkFailedVal(wristPID.getP(Constants.WRIST_PID_SLOT_POSITION), Constants.WRIST_PID1_P)){
        errorCount++;
      }
      wristPID.setI(Constants.WRIST_PID1_I, Constants.WRIST_PID_SLOT_POSITION);
      if(checkFailedVal(wristPID.getI(Constants.WRIST_PID_SLOT_POSITION), Constants.WRIST_PID1_I)){
        errorCount++;
      }
      wristPID.setD(Constants.WRIST_PID1_D, Constants.WRIST_PID_SLOT_POSITION);
      if(checkFailedVal(wristPID.getD(Constants.WRIST_PID_SLOT_POSITION), Constants.WRIST_PID1_D)){
        errorCount++;
      }
      wristPID.setIZone(Constants.WRIST_PID1_IZ, Constants.WRIST_PID_SLOT_POSITION);
      if(checkFailedVal(wristPID.getIZone(Constants.WRIST_PID_SLOT_POSITION), Constants.WRIST_PID1_IZ)){
        errorCount++;
      }
      wristPID.setFF(Constants.WRIST_PID1_F, Constants.WRIST_PID_SLOT_POSITION);
      if(checkFailedVal(wristPID.getFF(Constants.WRIST_PID_SLOT_POSITION), Constants.WRIST_PID1_F)){
        errorCount++;
      }
      wristPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.WRIST_PID_SLOT_POSITION);
      if(checkFailedVal(wristPID.getOutputMax(Constants.WRIST_PID_SLOT_POSITION), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if(checkFailedVal(wristPID.getOutputMin(Constants.WRIST_PID_SLOT_POSITION), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    
    if(errorCount == 0){
      success = true;
      break;
    }
   }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      int errorCount = 0;
      wristPID.setP(Constants.WRIST_PID2_P, Constants.WRIST_PID_SLOT_VELOCITY);
      if(checkFailedVal(wristPID.getP(Constants.WRIST_PID_SLOT_VELOCITY), Constants.WRIST_PID2_P)){
        errorCount++;
      }
      wristPID.setI(Constants.WRIST_PID2_I, Constants.WRIST_PID_SLOT_VELOCITY);
      if(checkFailedVal(wristPID.getI(Constants.WRIST_PID_SLOT_VELOCITY), Constants.WRIST_PID2_I)){
        errorCount++;
      }
      wristPID.setD(Constants.WRIST_PID2_D, Constants.WRIST_PID_SLOT_VELOCITY);
      if(checkFailedVal(wristPID.getD(Constants.WRIST_PID_SLOT_VELOCITY), Constants.WRIST_PID2_D)){
        errorCount++;
      }
      wristPID.setIZone(Constants.WRIST_PID2_IZ, Constants.WRIST_PID_SLOT_VELOCITY);
      if(checkFailedVal(wristPID.getD(Constants.WRIST_PID_SLOT_VELOCITY), Constants.WRIST_PID2_IZ)){
        errorCount++;
      }
      wristPID.setFF(Constants.WRIST_PID2_F, Constants.WRIST_PID_SLOT_VELOCITY);
      if(checkFailedVal(wristPID.getFF(Constants.WRIST_PID_SLOT_VELOCITY), Constants.WRIST_PID2_F)){
        errorCount++;
      }

      wristPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.WRIST_PID_SLOT_VELOCITY);
      if(checkFailedVal(wristPID.getOutputMax(Constants.WRIST_PID_SLOT_VELOCITY), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if(checkFailedVal(wristPID.getOutputMin(Constants.WRIST_PID_SLOT_VELOCITY), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    if(errorCount == 0){
      success = true;
      break;
    }
   }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for(int i = 0; i < 5; i++){
      wristPIDController.setTolerance(Constants.WRIST_POSITION_TOLERANCE);
      if(!checkFailedVal(wristPIDController.getPositionTolerance(), Constants.WRIST_POSITION_TOLERANCE)){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }
    
    resetWristAngle();

    success = false;
    for(int i = 0; i < 5; i++){
      if(wristMotor.burnFlash() == REVLibError.kOk){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
      System.out.println("Error at line Wrist: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

  stopWrist();
  SmartDashboard.putBoolean("Wrist Initialize", initGood);
  }

  public double getWristAngle() {
    return wristEncoder.getPosition();
  }    

public double getWristVelocity() {
    return wristEncoder.getVelocity();
}

public double getWristAbsoluteEncoderPosition() {
  double position = wristAbsoluteEncoder.getPosition();
   /*  if (position > Constants.WRIST_ABSOLUTE_ENCODER_ROLLOVER_THRESHOLD){
      position = position - Constants.WRIST_ABSOLUTE_ENCODER_ROLLOVER;
    }*/
      return position;
}

public void disableWristSoftLimits() {
    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
}

public void enableWristSoftLimits() {
    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
}

public void resetWristAngle(){
    //wristEncoder.setPosition(0);
    wristEncoder.setPosition(getWristAbsoluteEncoderPosition());
}

public void syncWristAbsAngle(ArmProfileSubsystem armProfile){
  if(Math.abs(getWristVelocity())< Constants.WRIST_ENCODER_RESET_MAX_VELOCITY && armProfile.getArmState() == ArmStates.STOW){
    wristSyncCounter++;
    if((wristSyncCounter >= Constants.ENCODER_RESET_ITERATIONS) || (lastArmState != armProfile.getArmState() && wristSyncCounter >= Constants.ENCODER_RESET_ITERATIONS_CHANGE_STATE)){
      lastArmState = armProfile.getArmState();
      wristSyncCounter = 0;
      wristEncoder.setPosition(getWristAbsoluteEncoderPosition());
    } 
  } else {
    wristSyncCounter = 0;
  }
}

public void stopWrist(){
  gotoWristAngle(6);
    /*double shoulderAngle = getShoulderAngle();
    double arbFF = wristFF.calculate(Units.degreesToRadians(wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 +25), 0); //FIXME Magic Numbers
    SmartDashboard.putNumber("Wrist Angle Calculated", (wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 + 25)); //FIXME Magic Numbers
    SmartDashboard.putNumber("arbFF Wrist", arbFF);     
    wristPID.setReference(wristAngle, ControlType.kPosition, Constants.WRIST_PID_SLOT_POSITION, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);*/
}

public void setWristSpeed(double speed) {
   
    wristMotor.set(speed);
    wristAngle = getWristAngle();

  }
  public void useOutputVoltage(double output, TrapezoidProfile.State setpoint, ShoulderSubsystem shoulder) {
    // Calculate the feedforward from the sepoint
    double shoulderAngle = shoulder.getShoulderAngle();
    //double feedforward = extendFF.calculate(setpoint.position, setpoint.velocity);
    double arbFF = wristFF.calculate(Units.degreesToRadians(shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 90), Units.degreesToRadians(setpoint.velocity)); 
    // Add the feedforward to the PID output to get the motor output
    wristMotor.setVoltage(output + arbFF);
  }

  public void useOutputPosition(double output, TrapezoidProfile.State setpoint, ShoulderSubsystem shoulder) {
    // Calculate the feedforward from the sepoint
    double shoulderAngle = shoulder.getShoulderAbsoluteEncoderPosition();
    double angle = getWristAbsoluteEncoderPosition() + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET;

    SmartDashboard.putNumber("Wrist Abs Angle", angle);

    //double feedforward = extendFF.calculate(setpoint.position, setpoint.velocity);
    double arbFF = wristFF.calculate(Units.degreesToRadians(angle), Units.degreesToRadians(setpoint.velocity)); 
    // Add the feedforward to the PID output to get the motor output
    SmartDashboard.putNumber("Wrist ArbFF", arbFF);
    wristPID.setReference(setpoint.position, ControlType.kPosition, Constants.WRIST_PID_SLOT_POSITION, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

public void setWristVelocity(double speed) {
 /*double shoulderAngle = getShoulderAngle();
    if(speed == 0){
      stopWrist();
    }else{
      double arbFF = wristFF.calculate(Units.degreesToRadians(wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 +25), Units.degreesToRadians(speed)); 
      wristPID.setReference(speed, ControlType.kVelocity, Constants.WRIST_PID_SLOT_VELOCITY, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
      SmartDashboard.putNumber("arbFF Wrist", arbFF);     
    }
    wristAngle = getWristAngle();*/

    /*double arbFF = wristFF2.calculate(speed); 
    wristPID.setReference(speed, ControlType.kVelocity, Constants.WRIST_PID_SLOT_VELOCITY, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
    SmartDashboard.putNumber("arbFF Wrist", arbFF);     
    SmartDashboard.putNumber("Wrist Target Speed", speed);   */  

  }
  

public void setWristVelocity2(double[] trajectoryPoint) {
    /*double shoulderAngle = getShoulderAngle();
    if(trajectoryPoint[Constants.TRAJECTORY_VELOCITY_INDEX] == 0){
      stopWrist();
    }else{

      double error = trajectoryPoint[Constants.TRAJECTORY_POSITION_INDEX] - getWristAbsoluteEncoderPosition();
     // double arbFF = wristFF.calculate(Units.degreesToRadians(wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 +25), Units.degreesToRadians(trajectoryPoint[Constants.TRAJECTORY_VELOCITY_INDEX]), Units.degreesToRadians(trajectoryPoint[Constants.TRAJECTORY_ACCELERATION_INDEX])) + Constants.WRIST_PID_PROFILED_P * error; 
      //wristPID.setReference(trajectoryPoint[Constants.TRAJECTORY_VELOCITY_INDEX], ControlType.kVelocity, Constants.WRIST_PID_SLOT_VELOCITY, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
      double arbFF = wristFF2.calculate(trajectoryPoint[Constants.TRAJECTORY_VELOCITY_INDEX])- 0.01 * error; 
      wristPID.setReference(trajectoryPoint[Constants.TRAJECTORY_VELOCITY_INDEX], ControlType.kVelocity, Constants.WRIST_PID_SLOT_VELOCITY, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
      SmartDashboard.putNumber("arbFF Wrist", arbFF);     
      SmartDashboard.putNumber("Wrist Target Speed", trajectoryPoint[Constants.TRAJECTORY_VELOCITY_INDEX]);     
  
    }
    wristAngle = getWristAngle();*/

  }


  public void gotoWristAngle(double angle) {
    wristPID.setReference(angle, ControlType.kPosition, Constants.WRIST_PID_SLOT_POSITION);  
}

/*public void setWristAngle(double targetAngle) {
    double shoulderAngle = getShoulderAngle();
    double arbFF = wristFF.calculate(Units.degreesToRadians(wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 +25), 0); 
    SmartDashboard.putNumber("Wrist Angle Calculated", (wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 + 25));
    SmartDashboard.putNumber("arbFF Wrist", arbFF);     
    wristPID.setReference(targetAngle, ControlType.kPosition, Constants.WRIST_PID_SLOT_POSITION, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
    wristAngle = targetAngle;
    wristMotor.set(0);
  }*/

public void setWristAngleTrapezoid(double targetAngle) {
    /*double shoulderAngle = getShoulderAngle();
    double arbFF = wristFF.calculate(Units.degreesToRadians(wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 +25), 0); 
    SmartDashboard.putNumber("Wrist Angle Calculated", (wristAngle + Constants.WRIST_ANGLE_OFFSET - shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET + 18 + 25));
    SmartDashboard.putNumber("arbFF Wrist", arbFF);     
    wristPID.setReference(targetAngle, ControlType.kPosition, Constants.WRIST_PID_SLOT_SMART_MOTION, arbFF, SparkMaxPIDController.ArbFFUnits.kVoltage);
    wristAngle = targetAngle;*/
  }


  public WristSubsystem(){
    setupMotors();
  }

  
  @Override
  public void periodic() {

    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putNumber("Wrist Absolute Angle", getWristAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Wrist Vel", getWristVelocity());
    SmartDashboard.putNumber("Wrist Abs Vel", wristAbsoluteEncoder.getVelocity());
    SmartDashboard.putNumber("Current Wrist", wristMotor.getOutputCurrent());

  }

  private boolean checkFailedVal(double x, double y) {
    double z = x - y;
    if (z == 0) {
      return false;
    } else {
      double decimalPlaces = (int) -Math.log10(x);
      return !(Math.floor(Math.abs(z * Math.pow(10, decimalPlaces + 1))) == 0);
    }
  }



    
}
