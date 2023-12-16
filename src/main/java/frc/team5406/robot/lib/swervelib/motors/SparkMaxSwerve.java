package frc.team5406.robot.lib.swervelib.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.team5406.robot.Constants;
import frc.team5406.robot.lib.swervelib.encoders.SwerveAbsoluteEncoder;
import frc.team5406.robot.lib.swervelib.parser.PIDFConfig;

/**
 * An implementation of {@link CANSparkMax} as a {@link SwerveMotor}.
 */
public class SparkMaxSwerve extends SwerveMotor
{

  /**
   * SparkMAX Instance.
   */
  public  CANSparkMax           motor;
  /**
   * Integrated encoder.
   */
  public  RelativeEncoder       encoder;
  /**
   * Absolute encoder attached to the SparkMax (if exists)
   */
  public  AbsoluteEncoder       absoluteEncoder;
  /**
   * Closed-loop PID controller.
   */
  public  SparkMaxPIDController pid;
  /**
   * Factory default already occurred.
   */
  private boolean               factoryDefaultOccurred = false;

  /**
   * Initialize the swerve motor.
   *
   * @param motor        The SwerveMotor as a SparkMax object.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   */

  private boolean error = false; 
  public SparkMaxSwerve(CANSparkMax motor, boolean isDriveMotor)
  {
    this.motor = motor;
    this.isDriveMotor = isDriveMotor;
    factoryDefaults();
    clearStickyFaults();

    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    pid.setFeedbackDevice(
        encoder); // Configure feedback of the PID controller as the integrated encoder.

    motor.setCANTimeout(0); // Spin off configurations in a different thread.
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link CANSparkMax} connected to a Brushless Motor.
   *
   * @param id           CAN ID of the SparkMax.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   */
  public SparkMaxSwerve(int id, boolean isDriveMotor)
  {
    this(new CANSparkMax(id, MotorType.kBrushless), isDriveMotor);
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      motor.enableVoltageCompensation(nominalVoltage); 
      if(motor.getVoltageCompensationNominalVoltage() == nominalVoltage){
        success = true;
        break;
      } 
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;
    }
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   * 
   * FIXME: Unable to validate answer.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit)
  {
    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      if(motor.setSmartCurrentLimit(currentLimit) == REVLibError.kOk){
        success = true;
        break;
      }
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;}
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      int errorCheck = 0;
      motor.setOpenLoopRampRate(rampRate);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(motor.getOpenLoopRampRate(), rampRate)){
        errorCheck++;
      }
      motor.setClosedLoopRampRate(rampRate);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(motor.getClosedLoopRampRate(), rampRate)){
        errorCheck++;
      }
      if(errorCheck == 0){
        success = true;
        break;
      }
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;
    }
  }

  /**
   * Get the motor object from the module.
   *
   * @return Motor object.
   */
  @Override
  public Object getMotor()
  {
    return motor;
  }

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  @Override
  public boolean isAttachedAbsoluteEncoder()
  {
    return absoluteEncoder != null;
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    boolean success = false;

    if (!factoryDefaultOccurred)
    {
      for(int i =0; i<Constants.LOOP_ITERATOR; i++){
        if(motor.restoreFactoryDefaults() == REVLibError.kOk){
          success = true;
          break;
        } 
        System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
        Timer.delay(Constants.CAN_RETRY_DELAY);
      }
      if(!success) {
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
        error = true;}
      factoryDefaultOccurred = true;
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      if(motor.clearFaults() == REVLibError.kOk){
        success = true;
        break;
      } 
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success){
    System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
    error = true;
  }
  }

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   * @return The {@link SwerveMotor} for easy instantiation.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    if (encoder.getAbsoluteEncoder() instanceof AbsoluteEncoder)
    {
      absoluteEncoder = (AbsoluteEncoder) encoder.getAbsoluteEncoder();
      pid.setFeedbackDevice(absoluteEncoder);
    }
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply.
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    if (absoluteEncoder == null)
    {
      boolean success = false;
      for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
        int errorCount = 0;
        encoder.setPositionConversionFactor(positionConversionFactor);
        Timer.delay(Constants.CAN_RETRY_DELAY);
        if(checkValMatch(encoder.getPositionConversionFactor(), positionConversionFactor)){
          errorCount++;
        }
        encoder.setVelocityConversionFactor(positionConversionFactor / 60);
        Timer.delay(Constants.CAN_RETRY_DELAY);
        if(checkValMatch(encoder.getVelocityConversionFactor(), (positionConversionFactor/60))){
          errorCount++;
        }   
        if(errorCount == 0){
          success = true;
          break;
        } 
        System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
        Timer.delay(Constants.CAN_RETRY_DELAY);
      }
      if(!success) {
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
        error = true;
      }


      // Taken from
      // https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/CANSparkMaxUtil.java#L67
      configureCANStatusFrames(10, 20, 20, 500, 500);
    } else
    {
      absoluteEncoder.setPositionConversionFactor(positionConversionFactor);
      absoluteEncoder.setVelocityConversionFactor(positionConversionFactor / 60);
    }
  }

  /**
   * Configure the PIDF values for the closed loop controller.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    int pidSlot =
        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();

    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      int errorCount = 0;
      pid.setP(config.p, pidSlot);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getP(pidSlot), config.p)){
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
        errorCount++;
      }
      pid.setI(config.i, pidSlot);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getI(pidSlot), config.i)){
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());

        errorCount++;
      }
      pid.setD(config.d, pidSlot);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getD(pidSlot), config.d)){
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());

        errorCount++;
      }
      pid.setFF(config.f, pidSlot);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getFF(pidSlot), config.f)){
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());

        errorCount++;
      }
      pid.setIZone(config.iz, pidSlot);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getIZone(pidSlot), config.iz)){
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());

        errorCount++;
      }
      pid.setOutputRange(config.output.min, config.output.max, pidSlot);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getOutputMax(pidSlot), config.output.max)){
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());

        errorCount++;
      }
      if(checkValMatch(pid.getOutputMin(pidSlot), config.output.min)){
        System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());

        errorCount++;
      }
      if(errorCount == 0){
        success = true;
        break;
      }
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
  }
  if(!success) {
    System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
    error = true;}

  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput)
  {
    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      int errorCheck = 0;
      pid.setPositionPIDWrappingEnabled(true);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(!pid.getPositionPIDWrappingEnabled()){
        errorCheck++;
      }
      pid.setPositionPIDWrappingMinInput(minInput);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getPositionPIDWrappingMinInput(), minInput)){
        errorCheck++;
      }
      pid.setPositionPIDWrappingMaxInput(maxInput);
      Timer.delay(Constants.CAN_RETRY_DELAY);
      if(checkValMatch(pid.getPositionPIDWrappingMaxInput(), maxInput)){
        errorCheck++;
      }
      if(errorCheck == 0){
        success = true;
        break;
      } 
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;
    }


  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
   * @param CANStatus2 Motor Position
   * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
   * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   * 
   * FIXME: Unable to validate answer.
   */
  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4)
  {
    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      int errorCheck = 0;
      if(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0) != REVLibError.kOk){
        errorCheck++;
      }
      if(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1) != REVLibError.kOk){
        errorCheck++;
      }
      if(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2) != REVLibError.kOk){
        errorCheck++;
      }
      if(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3) != REVLibError.kOk){
        errorCheck++;
      }
      if(motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4) != REVLibError.kOk){
        errorCheck++;
      }
      if(errorCheck == 0){
        success = true;
        break;
      } 
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;
    }

    // TODO: Configure Status Frame 5 and 6 if necessary
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    boolean success = false;
    IdleMode mode = isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      motor.setIdleMode(mode);
      if(motor.getIdleMode() == mode){
        success = true;
        break;
      } 
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;
    }
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    boolean success = false;
    for(int i =0; i<Constants.LOOP_ITERATOR; i++){
      motor.setInverted(inverted);
      if(motor.getInverted() == inverted){
        success = true;
        break;
      }
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;
    }
  }

  /**
   * Save the configurations from flash to EEPROM.
   * 
   * FIXME: Unable to validate answer.
   */
  @Override
  public void burnFlash()
  {
    boolean success = false;
    for(int i = 0; i<Constants.LOOP_ITERATOR; i++){
      if(motor.burnFlash() == REVLibError.kOk){
        success = true;
        break;
      }
      System.out.println("Failed to set " + i + ": " + new Throwable().getStackTrace()[0].getLineNumber());
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if(!success) {
      System.out.println("Error at line Swerve Module: " + new Throwable().getStackTrace()[0].getLineNumber());
      error = true;
    }
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput)
  {
    motor.set(percentOutput);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward)
  {
    int pidSlot =
        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    pid.setReference(
        setpoint,
        isDriveMotor ? ControlType.kVelocity : ControlType.kPosition,
        pidSlot,
        feedforward);
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    return absoluteEncoder == null ? encoder.getVelocity() : absoluteEncoder.getVelocity();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition()
  {
    return absoluteEncoder == null ? encoder.getPosition() : absoluteEncoder.getPosition();
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position.
   */
  @Override
  public void setPosition(double position)
  {
    if (absoluteEncoder == null)
    {
      encoder.setPosition(position);
    }
  }

  /**
   * REV Slots for PID configuration.
   */
  enum SparkMAX_slotIdx
  {
    /**
     * Slot 1, used for position PID's.
     */
    Position,
    /**
     * Slot 2, used for velocity PID's.
     */
    Velocity,
    /**
     * Slot 3, used arbitrarily. (Documentation show simulations).
     */
    Simulation
  }
  
  @Override
  public void setReference(double setpoint, double feedforward, double position)
  {
    setReference(setpoint, feedforward);
  }


  public boolean swerveCheck(){
    return error;
  }

  private boolean checkValMatch(double x, double y){
    double z = x - y;
    if(z == 0){
      return false;
    } else {
      double decimalPlaces = (int)-Math.log10(x);
      return !(Math.floor(Math.abs(z * Math.pow(10, decimalPlaces + 1))) == 0);
    }
  }

}
