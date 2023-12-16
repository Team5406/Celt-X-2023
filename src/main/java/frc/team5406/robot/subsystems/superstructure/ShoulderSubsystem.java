package frc.team5406.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

public class ShoulderSubsystem extends SubsystemBase {

  private CANSparkMax shoulderMotor = new CANSparkMax(Constants.SHOULDER_MOTOR_ONE, MotorType.kBrushless);
  private CANSparkMax shoulderMotorFollower = new CANSparkMax(Constants.SHOULDER_MOTOR_TWO, MotorType.kBrushless);
  private RelativeEncoder shoulderEncoder;
  private AbsoluteEncoder shoulderAbsoluteEncoder;
  private SparkMaxPIDController shoulderPID;
  ArmFeedforward shoulderFF = new ArmFeedforward(Constants.SHOULDER_KS, Constants.SHOULDER_KG, Constants.SHOULDER_KV,
      Constants.SHOULDER_KA);
  ArmFeedforward shoulderFFTrajectory = new ArmFeedforward(Constants.SHOULDER_KS, Constants.SHOULDER_KG,
      Constants.SHOULDER_KV, Constants.SHOULDER_KA);

  private double shoulderAngle = 0;

  private int shoulderSyncCounter;
  private boolean shoulderHome = false;
  public boolean initGood = true;
  public boolean success = false;

  private ArmStates lastArmState = ArmStates.STOW;

  public void setupMotors() {

    success = false;
    for (int i = 0; i < 5; i++) {
      int errorCount = 0;
      if (shoulderMotor.setCANTimeout(50) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 250) != REVLibError.kOk) {
        errorCount++;
      }
      if (shoulderMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 250) != REVLibError.kOk) {
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

      if (errorCount == 0) {
        success = true;
        break;
      }
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      if (shoulderMotor.restoreFactoryDefaults() == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      if ( shoulderMotorFollower.restoreFactoryDefaults() == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      if (shoulderMotorFollower.follow(shoulderMotor, Constants.SHOULDER_MOTOR_FOLLOWER_INVERSION) == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      shoulderMotor.setInverted(Constants.SHOULDER_MOTOR_INVERSION);
      if (shoulderMotor.getInverted() == Constants.SHOULDER_MOTOR_INVERSION) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      shoulderMotor.setIdleMode(IdleMode.kBrake);
      if (shoulderMotor.getIdleMode() == IdleMode.kBrake){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    
    success = false;
    for (int i = 0; i < 5; i++) {
      shoulderMotorFollower.setIdleMode(IdleMode.kBrake);
      if (shoulderMotorFollower.getIdleMode() == IdleMode.kBrake){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    shoulderEncoder = shoulderMotor.getEncoder();
    shoulderAbsoluteEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderPID = shoulderMotor.getPIDController();

    success = false;
    for (int i = 0; i < 5; i++) {
      if (shoulderPID.setFeedbackDevice(shoulderAbsoluteEncoder) == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      if (shoulderMotor.enableSoftLimit(SoftLimitDirection.kForward, false) == REVLibError.kOk
          && shoulderMotor.enableSoftLimit(SoftLimitDirection.kReverse, false) == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      if (shoulderMotorFollower.setSmartCurrentLimit(Constants.SHOULDER_CURRENT_LIMIT) == REVLibError.kOk
          && shoulderMotor.setSmartCurrentLimit(Constants.SHOULDER_CURRENT_LIMIT) == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      int errorCount = 0;
      shoulderEncoder.setPositionConversionFactor(Constants.DEGREES_PER_ROTATION / Constants.SHOULDER_GEAR_RATIO);
      if (checkFailedVal(shoulderEncoder.getPositionConversionFactor(), (Constants.DEGREES_PER_ROTATION / Constants.SHOULDER_GEAR_RATIO))) {
        errorCount++;
      }
      shoulderAbsoluteEncoder.setPositionConversionFactor(Constants.SHOULDER_ABSOLUTE_ENCODER_GEAR_RATIO * Constants.SHOULDER_GEAR_RATIO);
      if (checkFailedVal(shoulderAbsoluteEncoder.getPositionConversionFactor(), (Constants.SHOULDER_ABSOLUTE_ENCODER_GEAR_RATIO * Constants.SHOULDER_GEAR_RATIO))) {
        errorCount++;
      }
      shoulderEncoder.setVelocityConversionFactor(Constants.DEGREES_PER_ROTATION / (Constants.SHOULDER_GEAR_RATIO * Constants.SECONDS_PER_MINUTE));
      if (checkFailedVal(shoulderEncoder.getVelocityConversionFactor(), (Constants.DEGREES_PER_ROTATION / (Constants.SHOULDER_GEAR_RATIO * Constants.SECONDS_PER_MINUTE)))) {
        errorCount++;
      }
      shoulderAbsoluteEncoder.setInverted(true);
      if (shoulderAbsoluteEncoder.getInverted() != true) {
        errorCount++;
      }
      shoulderAbsoluteEncoder.setZeroOffset(Constants.SHOULDER_ABSOLUTE_ENCODER_OFFSET);
      if (checkFailedVal(shoulderAbsoluteEncoder.getZeroOffset(), (Constants.SHOULDER_ABSOLUTE_ENCODER_OFFSET))) {
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

      if (errorCount == 0) {
        success = true;
        break;
      }
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      int errorCount = 0;
      shoulderPID.setP(Constants.SHOULDER_PID0_P, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(shoulderPID.getP(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_PID0_P)) {
        errorCount++;
      }
      shoulderPID.setI(Constants.SHOULDER_PID0_I, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(shoulderPID.getI(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_PID0_I)) {
        errorCount++;
      }
      shoulderPID.setD(Constants.SHOULDER_PID0_D, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(shoulderPID.getD(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_PID0_D)) {
        errorCount++;
      }
      shoulderPID.setIZone(Constants.SHOULDER_PID0_IZ, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(shoulderPID.getIZone(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_PID0_IZ)) {
        errorCount++;
      }
      shoulderPID.setFF(Constants.SHOULDER_PID0_F, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if (checkFailedVal(shoulderPID.getFF(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_PID0_F)) {
        errorCount++;
      }
      shoulderPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(shoulderPID.getOutputMax(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if(checkFailedVal(shoulderPID.getOutputMin(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

      if (errorCount == 0) {
        success = true;
        break;
      }
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      int errorCount = 0;
      shoulderPID.setSmartMotionMaxVelocity(Constants.SHOULDER_MAX_SPEED, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(shoulderPID.getSmartMotionMaxVelocity(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_MAX_SPEED)){
        errorCount++;
      }
      shoulderPID.setSmartMotionMaxAccel(Constants.SHOULDER_MAX_ACCELERATION, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(shoulderPID.getSmartMotionMaxAccel(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_MAX_ACCELERATION)){
        errorCount++;
      }
      shoulderPID.setSmartMotionAllowedClosedLoopError(Constants.SHOULDER_ALLOWED_ERROR, Constants.SHOULDER_PID_SLOT_SMART_MOTION);
      if(checkFailedVal(shoulderPID.getSmartMotionAllowedClosedLoopError(Constants.SHOULDER_PID_SLOT_SMART_MOTION), Constants.SHOULDER_ALLOWED_ERROR)){
        errorCount++;
      }
      if(errorCount == 0){
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      int errorCount = 0;
      shoulderPID.setP(Constants.SHOULDER_PID1_P, Constants.SHOULDER_PID_SLOT_POSITION);
      if (checkFailedVal(shoulderPID.getP(Constants.SHOULDER_PID_SLOT_POSITION), Constants.SHOULDER_PID1_P)) {
        errorCount++;
      }
      shoulderPID.setI(Constants.SHOULDER_PID1_I, Constants.SHOULDER_PID_SLOT_POSITION);
      if (checkFailedVal(shoulderPID.getI(Constants.SHOULDER_PID_SLOT_POSITION), Constants.SHOULDER_PID1_I)) {
        errorCount++;
      }
      shoulderPID.setD(Constants.SHOULDER_PID1_D, Constants.SHOULDER_PID_SLOT_POSITION);
      if (checkFailedVal(shoulderPID.getD(Constants.SHOULDER_PID_SLOT_POSITION), Constants.SHOULDER_PID1_D)) {
        errorCount++;
      }
      shoulderPID.setIZone(Constants.SHOULDER_PID1_IZ, Constants.SHOULDER_PID_SLOT_POSITION);
      if (checkFailedVal(shoulderPID.getIZone(Constants.SHOULDER_PID_SLOT_POSITION), Constants.SHOULDER_PID1_IZ)) {
        errorCount++;
      } 
      shoulderPID.setFF(Constants.SHOULDER_PID1_F, Constants.SHOULDER_PID_SLOT_POSITION);
      if (checkFailedVal(shoulderPID.getFF(Constants.SHOULDER_PID_SLOT_POSITION), Constants.SHOULDER_PID1_F)) {
        errorCount++;
      }
      shoulderPID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.SHOULDER_PID_SLOT_POSITION);
      if(checkFailedVal(shoulderPID.getOutputMax(Constants.SHOULDER_PID_SLOT_POSITION), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if(checkFailedVal(shoulderPID.getOutputMin(Constants.SHOULDER_PID_SLOT_POSITION), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

      if (errorCount == 0) {
        success = true;
        break;
      }
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    resetShoulderAngle();

    success = false;
    for (int i = 0; i < 5; i++) {
      if (shoulderMotor.burnFlash() == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    success = false;
    for (int i = 0; i < 5; i++) {
      if (shoulderMotorFollower.burnFlash() == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Shoulder: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    stopShoulder();
    SmartDashboard.putBoolean("Shoulder Initialize", initGood);

  }

  public double getShoulderAngle() {
    return shoulderEncoder.getPosition();
  }

  public double getShoulderVelocity() {
    return shoulderEncoder.getVelocity();
  }

  public double getShoulderAbsoluteEncoderPosition() {
    double position = shoulderAbsoluteEncoder.getPosition();
    return position;
  }

  public void disableShoulderSoftLimits() {
    disableShoulderForwardSoftLimit();
    disableShoulderReverseSoftLimit();
  }

  public void disableShoulderForwardSoftLimit() {
    shoulderMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
  }

  public void disableShoulderReverseSoftLimit() {
    shoulderMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public void enableShoulderSoftLimits() {
    enableShoulderForwardSoftLimit();
    enableShoulderReverseSoftLimit();
  }

  public void enableShoulderForwardSoftLimit() {
    shoulderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  public void enableShoulderReverseSoftLimit() {
    shoulderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void resetShoulderAngle() {
    // shoulderEncoder.setPosition(0);
    shoulderEncoder.setPosition(getShoulderAbsoluteEncoderPosition()); // FIXME: Didn't work when not in cradle
  }

  public void syncShoulderAbsAngle(ArmProfileSubsystem armProfile) {
    if (Math.abs(getShoulderVelocity()) < Constants.SHOULDER_ENCODER_RESET_MAX_VELOCITY
        && armProfile.getArmState() == ArmStates.STOW) {
      shoulderSyncCounter++;
      if ((shoulderSyncCounter >= Constants.ENCODER_RESET_ITERATIONS) || (lastArmState != armProfile.getArmState()
          && shoulderSyncCounter >= Constants.ENCODER_RESET_ITERATIONS_CHANGE_STATE)) {
        lastArmState = armProfile.getArmState();
        shoulderSyncCounter = 0;
        shoulderEncoder.setPosition(getShoulderAbsoluteEncoderPosition());
      }
    } else {
      shoulderSyncCounter = 0;
    }
  }

  public void stopShoulder() {
    gotoShoulderAngle(5);
  }

  public void setShoulderSpeed(double RPM) {
    shoulderMotor.set(RPM);
    shoulderAngle = getShoulderAngle();

  }

  public void useOutputPosition(double output, TrapezoidProfile.State setpoint, ExtendSubsystem extend) {
    // Calculate the feedforward from the sepoint

    double shoulderAngle = getShoulderAbsoluteEncoderPosition();
    double angle = shoulderAngle + Constants.SHOULDER_ANGLE_OFFSET;

    double arbFF = shoulderFF.calculate(Units.degreesToRadians(angle), Units.degreesToRadians(setpoint.velocity));
    // Add the feedforward to the PID output to get the motor output
    shoulderPID.setReference(setpoint.position, ControlType.kPosition, Constants.SHOULDER_PID_SLOT_POSITION, arbFF,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public void useOutputVoltage(double output, TrapezoidProfile.State setpoint, ExtendSubsystem extend) {
    // Calculate the feedforward from the sepoint
    double shoulderAngle = getShoulderAngle();
    // double feedforward = extendFF.calculate(setpoint.position,
    // setpoint.velocity);
    double arbFF = extend.getExtendAbsolutePosition()
        * Math.cos(Units.degreesToRadians(Constants.SHOULDER_ANGLE_OFFSET + getShoulderAngle())) * 0.2
        + shoulderFF.calculate(Units.degreesToRadians(Constants.SHOULDER_ANGLE_OFFSET + getShoulderAngle()),
            Units.degreesToRadians(setpoint.velocity));
    // Add the feedforward to the PID output to get the motor output
    shoulderMotor.setVoltage(output + arbFF);
  }

  public void setShoulderVoltage(double voltage) {
    shoulderMotor.setVoltage(voltage);
    System.out.println("shoulder set");
  }

  public void gotoShoulderAngle(double angle) {
    shoulderPID.setReference(angle, ControlType.kPosition, Constants.SHOULDER_PID_SLOT_POSITION);
  }

  public ShoulderSubsystem() {
    setupMotors();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shoulder Vel", getShoulderVelocity());
    SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
    SmartDashboard.putNumber("Shoulder Absolute Angle", getShoulderAbsoluteEncoderPosition());
    SmartDashboard.putNumber("Current Shoulder", shoulderMotor.getOutputCurrent());
    SmartDashboard.putNumber("arbFF Shoulder", 0);

    if (shoulderEncoder.getVelocity() < 20 && shoulderAbsoluteEncoder.getPosition() < 20) {
      shoulderMotor.setSmartCurrentLimit(Constants.SHOULDER_LOW_CURRENT_LIMIT);
    } else {
      shoulderMotor.setSmartCurrentLimit(Constants.SHOULDER_CURRENT_LIMIT);
    }
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
