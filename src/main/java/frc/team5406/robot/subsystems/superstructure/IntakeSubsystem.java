/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team5406.robot.subsystems.superstructure;

import frc.team5406.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_ONE, MotorType.kBrushless);
  private RelativeEncoder intakeEncoder;
  private SparkMaxPIDController intakePID;
  SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(Constants.INTAKE_KS, Constants.INTAKE_KV,
      Constants.INTAKE_KA);
  static Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private PowerDistribution pdh = new PowerDistribution();
  private int intakeCurrentSpikeIterator;
  private boolean currentSpiked = false;

  boolean initGood = true;

  public enum GamePieces {
    CONE,
    CUBE,
    NONE,

  };

  public enum HoldingCurrent {
    CONE,
    CUBE,
    NONE,

  };

  private int existingCurrentLimit = 20;

  private GamePieces currentGamePiece = GamePieces.NONE;
  private HoldingCurrent currentHoldingCurrent = HoldingCurrent.NONE;

  public void setupMotors() {
    // intakeMotor.restoreFactoryDefaults();
    boolean success = false;

    success = false;
    for (int i = 0; i < 5; i++) {
      int errorCount = 0;
      if (intakeMotor.setCANTimeout(50) != REVLibError.kOk) {
        errorCount++;
      }
      if (intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10) != REVLibError.kOk) {
        errorCount++;
      }
      if (intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50) != REVLibError.kOk) {
        errorCount++;
      }
      if (intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50) != REVLibError.kOk) {
        errorCount++;
      }
      if (intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200) != REVLibError.kOk) {
        errorCount++;
      }
      if (intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000) != REVLibError.kOk) {
        errorCount++;
      }
      if (intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50) != REVLibError.kOk) {
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

      if (errorCount == 0) {
        success = true;
        break;
      }
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      if (intakeMotor.restoreFactoryDefaults() == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    // intakeMotor.setInverted(Constants.INTAKE_MOTOR_INVERSION);
    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      if (intakeMotor.getInverted() == Constants.INTAKE_MOTOR_INVERSION) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;

    }

    // intakeMotor.setIdleMode(IdleMode.kCoast);
    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      intakeMotor.setIdleMode(IdleMode.kCoast);
      if (intakeMotor.getIdleMode() ==IdleMode.kCoast) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;

    }

    intakeEncoder = intakeMotor.getEncoder();
    intakePID = intakeMotor.getPIDController();

    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      if (intakeMotor.enableSoftLimit(SoftLimitDirection.kForward, false) == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;

    }

    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      if (intakeMotor.enableSoftLimit(SoftLimitDirection.kReverse, false) == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;

    }

    // intakeMotor.setSmartCurrentLimit(existingCurrentLimit);
    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      if (intakeMotor.setSmartCurrentLimit(existingCurrentLimit) == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;

    }

    // intakeEncoder.setVelocityConversionFactor(1 / (Constants.INTAKE_GEAR_RATIO *
    // Constants.SECONDS_PER_MINUTE));
    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      intakeEncoder.setVelocityConversionFactor(1 / (Constants.INTAKE_GEAR_RATIO * Constants.SECONDS_PER_MINUTE));
      if (!checkFailedVal(intakeEncoder.getVelocityConversionFactor(), 1 / (Constants.INTAKE_GEAR_RATIO * Constants.SECONDS_PER_MINUTE))) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;

    }

    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      int errorCount = 0;
      intakePID.setP(Constants.INTAKE_PID0_P, Constants.INTAKE_PID_SLOT_VELOCITY);
      if (checkFailedVal(intakePID.getP(Constants.INTAKE_PID_SLOT_VELOCITY), Constants.INTAKE_PID0_P)) {
        errorCount++;
      }
      intakePID.setI(Constants.INTAKE_PID0_I, Constants.INTAKE_PID_SLOT_VELOCITY);
      if (checkFailedVal(intakePID.getI(Constants.INTAKE_PID_SLOT_VELOCITY), Constants.INTAKE_PID0_I)) {
        errorCount++;
      }
      intakePID.setD(Constants.INTAKE_PID0_D, Constants.INTAKE_PID_SLOT_VELOCITY);
      if (checkFailedVal(intakePID.getD(Constants.INTAKE_PID_SLOT_VELOCITY), Constants.INTAKE_PID0_D)) {
        errorCount++;
      }
      intakePID.setIZone(Constants.INTAKE_PID0_IZ, Constants.INTAKE_PID_SLOT_VELOCITY);
      if (checkFailedVal(intakePID.getIZone(Constants.INTAKE_PID_SLOT_VELOCITY), Constants.INTAKE_PID0_IZ)) {
        errorCount++;
      }
      intakePID.setFF(Constants.INTAKE_PID0_F, Constants.INTAKE_PID_SLOT_VELOCITY);
      if (checkFailedVal(intakePID.getFF(Constants.INTAKE_PID_SLOT_VELOCITY), Constants.INTAKE_PID0_F)) {
        errorCount++;
      }
      intakePID.setOutputRange(Constants.OUTPUT_RANGE_MIN, Constants.OUTPUT_RANGE_MAX, Constants.INTAKE_PID_SLOT_VELOCITY);
      if(checkFailedVal(intakePID.getOutputMax(Constants.INTAKE_PID_SLOT_VELOCITY), Constants.OUTPUT_RANGE_MAX)){
        errorCount++;
      }
      if(checkFailedVal(intakePID.getOutputMin(Constants.INTAKE_PID_SLOT_VELOCITY), Constants.OUTPUT_RANGE_MIN)){
        errorCount++;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);

      if (errorCount == 0) {
        success = true;
        break;
      }
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    // intakeMotor.burnFlash();

    success = false;
    for (int i = 0; i < Constants.LOOP_ITERATOR; i++) {
      if (intakeMotor.burnFlash() == REVLibError.kOk) {
        success = true;
        break;
      }
      Timer.delay(Constants.CAN_RETRY_DELAY);
    }
    if (!success) {
      System.out.println("Error at line Intake: " + new Throwable().getStackTrace()[0].getLineNumber());
      initGood = false;
    }

    SmartDashboard.putBoolean("Intake Init", initGood);

    setHeadlights(false);

  }

  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  public void stop() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);

    double voltage = 0;
    switch (currentHoldingCurrent) {
      case CONE:
        setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CONE);
        break;
      case CUBE:
        setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE);
        break;
      case NONE:
      default:
        setIntakeVoltage(0);
        break;
    }

  }

  public void stopHoldingCurrent() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    intakeMotor.set(0);
  }

  public void setVelocity(double velocity) {
    double arbFF;
    arbFF = intakeFF.calculate(velocity);

    SmartDashboard.putNumber("Intake arbFF", arbFF);
    SmartDashboard.putNumber("Intake velocity target", velocity);

    intakePID.setReference(velocity, ControlType.kVelocity, Constants.INTAKE_PID_SLOT_VELOCITY, arbFF,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
  }

  public void coneIntake() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    currentHoldingCurrent = HoldingCurrent.CONE;
    setHoldingCurrent(HoldingCurrent.CONE);
    setVelocity(Constants.INTAKE_SPEED_CONE);
    setGamePieceType(GamePieces.CONE);
  }

  public void cubeIntake() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    currentHoldingCurrent = HoldingCurrent.CUBE;
    setHoldingCurrent(HoldingCurrent.CUBE);
    setVelocity(Constants.INTAKE_SPEED_CUBE);
    setGamePieceType(GamePieces.CUBE);
  }

  public void setHoldingCurrent(HoldingCurrent gamePiece) {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    double currentSpike = 0, voltage;
    switch (gamePiece) {
      case CONE:
        setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CONE);
        break;
      case CUBE:
        setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE);
        break;
      case NONE:
      default:
        setIntakeVoltage(0);
    }

    if (Math.abs(intakeMotor.getOutputCurrent()) > currentSpike
        && Math.abs(getVelocity()) < Constants.INTAKE_VELOCITY_DEADBAND) {
      intakeCurrentSpikeIterator++;
      if (intakeCurrentSpikeIterator >= Constants.INTAKE_CURRENT_SPIKE_MAX_ITERATOR) {
        currentSpiked = true;
      }
    }
  }

  public void coneOuttake() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT_SPIKE);
    setVelocity(Constants.OUTTAKE_SPEED_CONE);
    currentHoldingCurrent = HoldingCurrent.NONE;

    currentSpiked = false;
    intakeCurrentSpikeIterator = 0;
  }

  public void cubeOuttake() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT_SPIKE);
    setVelocity(Constants.OUTTAKE_SPEED_CUBE);
    currentHoldingCurrent = HoldingCurrent.NONE;

    currentSpiked = false;
    intakeCurrentSpikeIterator = 0;
  }

  public void coneThrow() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT_SPIKE);
    setVelocity(Constants.THROW_SPEED_CONE);
    currentHoldingCurrent = HoldingCurrent.NONE;

    currentSpiked = false;
    intakeCurrentSpikeIterator = 0;
  }

  public void cubeThrow() {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT_SPIKE);
    setVelocity(Constants.THROW_SPEED_CUBE);
    currentHoldingCurrent = HoldingCurrent.NONE;

    currentSpiked = false;
    intakeCurrentSpikeIterator = 0;
  }

  public void setGamePieceType(GamePieces gamePiece) {
    currentGamePiece = gamePiece;
  }

  public void setCurrentLimit(int currentLimit) {
    if (currentLimit != existingCurrentLimit) {
      intakeMotor.setSmartCurrentLimit(currentLimit);
      existingCurrentLimit = currentLimit;
    }

  }

  public GamePieces getGamePieceType() {
    return currentGamePiece;
  }

  public static void compressorEnabled() {
    compressor.enableAnalog(Constants.MIN_PRESSURE, Constants.MAX_PRESSURE);
  }

  public static void compressorDisabled() {
    compressor.disable();
  }

  public static double getPressure() {
    return compressor.getPressure();
  }

  public IntakeSubsystem() {
    setupMotors();
    compressorEnabled();
  }

  public void setIntakeVoltage(double voltage) {
    setCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    intakeMotor.setVoltage(voltage);
  }

  public void setHeadlights(boolean state) {
    pdh.setSwitchableChannel(state);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Vel", getVelocity());
    SmartDashboard.putNumber("Pressure", getPressure());
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());
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
