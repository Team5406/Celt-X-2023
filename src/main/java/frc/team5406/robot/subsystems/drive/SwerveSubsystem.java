package frc.team5406.robot.subsystems.drive;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.lib.swervelib.SwerveController;
import frc.team5406.robot.lib.swervelib.SwerveDrive;
import frc.team5406.robot.lib.swervelib.math.SwerveKinematics2;
import frc.team5406.robot.lib.swervelib.math.SwerveModuleState2;
import frc.team5406.robot.lib.swervelib.parser.SwerveControllerConfiguration;
import frc.team5406.robot.lib.swervelib.parser.SwerveDriveConfiguration;
import frc.team5406.robot.lib.swervelib.parser.SwerveParser;
import frc.team5406.robot.lib.swervelib.telemetry.SwerveDriveTelemetry;
import frc.team5406.robot.lib.swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;
  private Rotation2d savedHeading;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg);
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   * @param isOpenLoop    Whether to use closed-loop velocity control. Set to true
   *                      to disable closed-loop.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    //System.out.println(translation.getX());
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    SmartDashboard.putNumber("Module 0 Drive Motor Position", swerveDrive.getDriveModulePosition()[0]);
    SmartDashboard.putNumber("Module 1 Drive Motor Position", swerveDrive.getDriveModulePosition()[1]);
    SmartDashboard.putNumber("Module 2 Drive Motor Position", swerveDrive.getDriveModulePosition()[2]);
    SmartDashboard.putNumber("Module 3 Drive Motor Position", swerveDrive.getDriveModulePosition()[3]);
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveKinematics2} of the swerve drive.
   */
  public SwerveKinematics2 getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void setGyro(double angle) {
    swerveDrive.setGyro(angle);
  }


  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW
   * positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  public void saveHeading() {
    savedHeading = getHeading();
    System.out.println(savedHeading.getDegrees());
  }

  public void restoreHeading() {
    setGyro(savedHeading.getDegrees());
    System.out.println(savedHeading.getDegrees());
  }


  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians());
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    swerveDrive.lockPose();
  }

  // Degrees
  public void setCardinalDirection(double rotation) {
    ChassisSpeeds setRotation = getTargetSpeeds(0, 0, new Rotation2d(Units.degreesToRadians(rotation)));
    // ChassisSpeeds robotVel = getFieldVelocity();
    swerveDrive.drive(SwerveController.getTranslation2d(setRotation), setRotation.omegaRadiansPerSecond, true, false);
  }

  public void setModuleStates(SwerveModuleState2[] desiredStates) {
    swerveDrive.setModuleStates(desiredStates, false);
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp(), false, 1);
  }

  public void addVisionMeasurement(Pose2d pose, double time) {
    swerveDrive.addVisionMeasurement(pose, time, true, 1);
  }

  public void setTurnStates(double speed) {
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, speed);
    setChassisSpeeds(speeds);

  }

  public void getConversionFactor(){
    System.out.println((Math.PI * swerveDrive.swerveDriveConfiguration.modules[0].configuration.physicalCharacteristics.wheelDiameter)/(swerveDrive.swerveDriveConfiguration.modules[0].configuration.physicalCharacteristics.driveGearRatio)) ;
  }

  public void syncAbsEncoders(){
    swerveDrive.synchronizeModuleEncoders();
  }
}