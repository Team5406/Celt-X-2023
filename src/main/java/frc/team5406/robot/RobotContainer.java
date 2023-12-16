// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5406.robot.autos.Balance;
import frc.team5406.robot.autos.ConeCubeConeMiddleBlue;
import frc.team5406.robot.autos.ConeCubeCubeMiddleRed;
import frc.team5406.robot.autos.ConeCubeEdgeBlue;
import frc.team5406.robot.autos.ConeCubeEdgeRed;
import frc.team5406.robot.autos.ConeCubeMiddleBlue;
import frc.team5406.robot.autos.ConeCubeMiddleRed;
import frc.team5406.robot.autos.ConeL3CubeBalanceBlue;
import frc.team5406.robot.autos.ConeL3CubeBalanceRed;
import frc.team5406.robot.autos.CubeL3Balance;
import frc.team5406.robot.autos.DoNothing;
import frc.team5406.robot.autos.DriveStraight;
import frc.team5406.robot.autos.ConeL3Straight;
import frc.team5406.robot.autos.L3CubeMobilityBalance;
import frc.team5406.robot.autos.StraightFind;

import frc.team5406.robot.commands.AlignWithDirection;
import frc.team5406.robot.commands.AlignWithGamePiece;
import frc.team5406.robot.commands.CancelScoringGamepiece;
import frc.team5406.robot.commands.DriveToConeNodeL2;
import frc.team5406.robot.commands.DriveToConeNodeL3;
import frc.team5406.robot.commands.DriveToCubeNode;
import frc.team5406.robot.commands.ExtendToL1;
import frc.team5406.robot.commands.ExtendToL2;
import frc.team5406.robot.commands.ExtendToL3;
import frc.team5406.robot.commands.IntakeConeFloor;
import frc.team5406.robot.commands.IntakeConeSubstation;
import frc.team5406.robot.commands.IntakeCubeFloor;
import frc.team5406.robot.commands.LimelightDrivecam;
import frc.team5406.robot.commands.TeleopDrive;
import frc.team5406.robot.commands.MoveWristTrapezoid;
import frc.team5406.robot.commands.OuttakeConeTeleop;
import frc.team5406.robot.commands.OuttakeCubeTeleop;
import frc.team5406.robot.commands.ScoreGamepieceL2;
import frc.team5406.robot.commands.ScoreGamepieceL3;
import frc.team5406.robot.commands.ShootCube;
import frc.team5406.robot.commands.SyncAbsEncodersSwerve;
import frc.team5406.robot.subsystems.vision.LimelightSubsystem;
import frc.team5406.robot.subsystems.vision.ObjectDetectionSubsystem;
import frc.team5406.robot.subsystems.drive.SwerveSubsystem;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem;
import frc.team5406.robot.subsystems.superstructure.ArmProfileSubsystem.ArmStates;
import frc.team5406.robot.subsystems.superstructure.IntakeSubsystem.GamePieces;
import frc.team5406.robot.subsystems.superstructure.ExtendSubsystem;
import frc.team5406.robot.subsystems.superstructure.ShoulderSubsystem;
import frc.team5406.robot.subsystems.superstructure.WristSubsystem;
import frc.team5406.robot.subsystems.superstructure.ArmProfileSubsystem;

public class RobotContainer {

    SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    IntakeSubsystem intake = new IntakeSubsystem();
    ExtendSubsystem extend = new ExtendSubsystem();
    ShoulderSubsystem shoulder = new ShoulderSubsystem();
    WristSubsystem wrist = new WristSubsystem();
    LimelightSubsystem limelight = new LimelightSubsystem();
    ObjectDetectionSubsystem orangepi = new ObjectDetectionSubsystem();
    ArmProfileSubsystem armProfile = new ArmProfileSubsystem();

    CommandBase driveStraight = DriveStraight.auto(swerve);
    CommandBase doNothing = DoNothing.auto();
    CommandBase balance = Balance.auto(swerve);
    CommandBase coneL3Straight = ConeL3Straight.auto(swerve, shoulder, extend, wrist, intake, limelight);
    CommandBase cubeL3Balance = CubeL3Balance.auto(swerve, shoulder, extend, wrist, intake, limelight);
    CommandBase coneL3CubeBalanceRed = ConeL3CubeBalanceRed.auto(swerve, shoulder, extend, wrist, intake, limelight, orangepi);
    CommandBase coneL3CubeBalanceBlue = ConeL3CubeBalanceBlue.auto(swerve, shoulder, extend, wrist, intake, limelight, orangepi);
    CommandBase coneCubeMiddleRed = ConeCubeMiddleRed.auto(swerve, shoulder, extend, wrist, intake, limelight);
    CommandBase coneCubeMiddleBlue = ConeCubeMiddleBlue.auto(swerve, shoulder, extend, wrist, intake, limelight);
    CommandBase coneCubeEdgeBlue = ConeCubeEdgeBlue.auto(swerve, shoulder, extend, wrist, intake, limelight, orangepi);
    CommandBase coneCubeEdgeRed = ConeCubeEdgeRed.auto(swerve, shoulder, extend, wrist, intake, limelight, orangepi);
    CommandBase l3CubeMobilityBalance = L3CubeMobilityBalance.auto(swerve, shoulder, extend, wrist, intake, limelight, orangepi);
    CommandBase straightFind = StraightFind.auto(swerve, shoulder, extend, wrist, intake, limelight, orangepi);
    CommandBase coneCubeCubeMiddleRed = ConeCubeCubeMiddleRed.auto(swerve, shoulder, extend, wrist, intake, limelight);
    CommandBase coneCubeConeMiddleBlue = ConeCubeConeMiddleBlue.auto(swerve, shoulder, extend, wrist, intake, limelight);




    CommandXboxController driverGamepad = new CommandXboxController(Constants.DRIVER_CONTROLLER);
    CommandXboxController operatorGamepad = new CommandXboxController(Constants.OPERATOR_CONTROLLER);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        /*
         * AbsoluteDrive driveCommmand = new AbsoluteDrive(
         * swerve,
         * () -> aboveDeadband(-driverGamepad.getLeftY()),
         * () -> aboveDeadband(-driverGamepad.getLeftX()),
         * () -> -driverGamepad.getRightX(),
         * () -> driverGamepad.getRightY(),
         * false);
         */

        TeleopDrive driveCommmand = new TeleopDrive(
                swerve,
                () -> aboveDeadband(-driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY())),
                () -> aboveDeadband(-driverGamepad.getLeftX()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY())),
                () -> -driverGamepad.getRightX(), () -> true, false, false);

        swerve.setDefaultCommand(driveCommmand);

        intake.setDefaultCommand(
                new RunCommand(intake::stop, intake));

        limelight.setDefaultCommand(
                new LimelightDrivecam(limelight)
        );
         
         m_chooser.setDefaultOption("2 - Drive Straight Auto", driveStraight);
         m_chooser.addOption("1 - Do Nothing", doNothing);
         m_chooser.addOption("3 - Balance", balance);
         m_chooser.addOption("4 - Cone L3 - Straight", coneL3Straight);
         m_chooser.addOption("5 - Cube L3 - Balance", cubeL3Balance);
         m_chooser.addOption("6 - Red - Cone, Cube - Right", coneCubeMiddleRed);
         m_chooser.addOption("7 - Red - Cone, Cube - Left", coneCubeEdgeRed);
         m_chooser.addOption("9 - Blue - Cone, Cube - Left", coneCubeMiddleBlue);
         m_chooser.addOption("10 - Blue - Cone, Cube - Right", coneCubeEdgeBlue);
         m_chooser.addOption("12 - Red - Cube L3 Cube Balance ", coneL3CubeBalanceRed);
         m_chooser.addOption("13 - Blue - Cube L3 Cube Balance ", coneL3CubeBalanceBlue);
         m_chooser.addOption("14 - Cube L3 Balance Mobility ", l3CubeMobilityBalance);
         m_chooser.addOption("16 - Reserved for Blue ", l3CubeMobilityBalance);
         m_chooser.addOption("17 - Cone (L3), Cube (L3), Cone Pickup - Red - Right", coneCubeCubeMiddleRed);
         m_chooser.addOption("18 -  Cone (L3), Cube (L3), Cone Pickup - Blue - Left", coneCubeConeMiddleBlue);
         m_chooser.addOption("99 - Drive Straight Find ", straightFind);
         SmartDashboard.putData(m_chooser);
    }

    private void configureBindings() {
        // zero Gyro
        driverGamepad.back().whileTrue(
                new InstantCommand(swerve::zeroGyro));

        //x-lock
        driverGamepad.x().
        whileTrue(new RepeatCommand(new InstantCommand(swerve::lock, swerve)));

        //stow
        operatorGamepad.b()
                .onTrue(
                        new ConditionalCommand(
                                new ExtendToL2(shoulder, extend, wrist, limelight, intake),
                                new ShootCube(shoulder, extend, wrist, intake),
                                () -> intake.getGamePieceType() == GamePieces.CONE
                        )
                );

        // home without scoring
        operatorGamepad.x()
                .onTrue(
                        new CancelScoringGamepiece(shoulder, extend, wrist)
                );

        // score and return home
        operatorGamepad.rightBumper()
                .onTrue(   
                        new ConditionalCommand(
                                new ScoreGamepieceL3(shoulder, extend, wrist, intake), 
                                new ScoreGamepieceL2(shoulder, extend, wrist, intake), 
                                () -> extend.getArmLevel() == ArmStates.L3)                
                );

        // L3
        operatorGamepad.y()
                .onTrue(
                        new ExtendToL3(shoulder, extend, wrist, limelight)
                );

                
        //Auto-Align with Cone
        driverGamepad.a().whileTrue(
                new RepeatCommand(
                new AlignWithGamePiece(swerve, orangepi, 
                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> -driverGamepad.getLeftX()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                true))
        );

        //Auto-Align with Cube
        driverGamepad.a().and(driverGamepad.rightBumper()).whileTrue(
                new RepeatCommand(
                new AlignWithGamePiece(swerve, orangepi, 
                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> -driverGamepad.getLeftX()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                false))
        );

       // Intake Cone
        driverGamepad.rightTrigger(Constants.CONTROLLER_ANALOG_THRESHOLD).and(driverGamepad.rightBumper().negate())
                .whileTrue(
                        new IntakeConeFloor(shoulder, extend, wrist, intake)
                )
                .onFalse(
                        new MoveWristTrapezoid(Constants.WRIST_HOME_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION));

        // Intake Cube
        driverGamepad.rightTrigger(Constants.CONTROLLER_ANALOG_THRESHOLD).and(driverGamepad.rightBumper())
                .whileTrue(
                        new IntakeCubeFloor(shoulder, extend, wrist, intake)
                ).onFalse(
                        new MoveWristTrapezoid(Constants.WRIST_HOME_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION));

        // Align to Cube Node
        driverGamepad.start().whileTrue(new DriveToCubeNode(swerve, limelight, () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY())
        ));
  
        //Manual Cube Intake
        operatorGamepad.leftTrigger(Constants.CONTROLLER_ANALOG_THRESHOLD)
        .whileTrue(
                new RunCommand(() -> intake.cubeIntake(), intake)
        );
        
        //Manual Cone Intake
        operatorGamepad.rightTrigger(Constants.CONTROLLER_ANALOG_THRESHOLD)
        .whileTrue(
                new RunCommand(() -> intake.coneIntake(), intake)
        );

        //Auto-Align L2
        driverGamepad.b().whileTrue(
                new RepeatCommand(

                                new DriveToConeNodeL2(swerve, limelight, 
                                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY())
                                ))
        );

        //Auto-Align L3
        driverGamepad.y().whileTrue(
                        new RepeatCommand(
                                new DriveToConeNodeL3(swerve, limelight, 
                                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY())
                                ))
        );

        // L1
        operatorGamepad.a()
                .onTrue(
                        new ExtendToL1(shoulder, extend, wrist, intake)
                ).onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> intake.stop(), intake),
                                new MoveWristTrapezoid(Constants.WRIST_HOME_POSITION, wrist, shoulder)
                        )
                );

        // Outake Cone
        driverGamepad.leftTrigger(Constants.CONTROLLER_ANALOG_THRESHOLD).and(driverGamepad.rightBumper().negate())
                .whileTrue(
                            new OuttakeConeTeleop(shoulder, extend, wrist, intake)
                        )
                                
                .onFalse(
                        new MoveWristTrapezoid(Constants.WRIST_HOME_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION));

        // Outake Cube
        driverGamepad.leftTrigger(Constants.CONTROLLER_ANALOG_THRESHOLD).and(driverGamepad.rightBumper())
                .whileTrue(
                       new OuttakeCubeTeleop(shoulder, extend, wrist, intake)
                                        )
                .onFalse(
                        new MoveWristTrapezoid(Constants.WRIST_HOME_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION));

        //Cardinal Direction Forward (Facing away from driver wall)
        driverGamepad.povUp().whileTrue(
                new AlignWithDirection(swerve,
                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> -driverGamepad.getLeftX()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> 0
                )
        );

        //Cardinal Direction Down (Facing towards driver wall)
        driverGamepad.povDown().whileTrue(
                new AlignWithDirection(swerve,
                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> -driverGamepad.getLeftX()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> 180
                )
        );

        //Cardinal Direction Left
        driverGamepad.povLeft().whileTrue(
                new AlignWithDirection(swerve,
                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> -driverGamepad.getLeftX()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> 90
                )
        );

        //Cardinal Direction Right
        driverGamepad.povRight().whileTrue(
                new AlignWithDirection(swerve,
                () -> -driverGamepad.getLeftY()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> -driverGamepad.getLeftX()/inputScale(driverGamepad.getLeftX(), driverGamepad.getLeftY()),
                () -> -90
                )
        );

        //Intake at Single Substation
        driverGamepad.leftBumper().onTrue(
                new IntakeConeSubstation(shoulder, extend, wrist, intake)
        ).onFalse(
                 new ParallelCommandGroup(
                        new MoveWristTrapezoid(Constants.WRIST_HOME_POSITION, wrist, shoulder, Constants.WRIST_EXTEND_FAST_ACCELERATION),
                        new InstantCommand(() -> intake.setHeadlights(false), intake)   
                        )  
                );

        //Fixes Swerve wheel drift
        //Fixed this instead by three way communication with CAN Devices
        driverGamepad.leftBumper().and(driverGamepad.b()).whileTrue(
                new SyncAbsEncodersSwerve(swerve)
        );
 
        //Offsets alignment for game pieces justified left in the intake
        operatorGamepad.povLeft().onTrue(
                new InstantCommand(() -> limelight.updatePos(180), limelight)
        );

        //Offsets alignment for game pieces centered in the intake
        operatorGamepad.povDown().onTrue(
                new InstantCommand(() -> limelight.updatePos(270), limelight)
        );

        //Offsets alignment for game pieces justified right in the intake
        operatorGamepad.povRight().onTrue(
                new InstantCommand(() -> limelight.updatePos(0), limelight)
        );

        
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    //Scales circular/diagonal inputs
    public double inputScale(double x, double y) {
        double xAbs = Math.abs(x);
        double yAbs = Math.abs(y);
        double angle = Math.atan2(yAbs, xAbs);
        double result = yAbs > xAbs ? Math.sin(angle) : Math.cos(angle);
        SmartDashboard.putNumber("input scale", result);
        return result;
    }

    public double aboveDeadband(double input) {
        return (Math.abs(input) > Constants.DEFAULT_DEADBAND) ? input : 0;
    }

    public void turnOnLED(){
        limelight.turnOnLED();
    }
    public void turnOffLED(){
        limelight.turnOffLED();
    }

    public void resetGyro(){
        swerve.setGyro(swerve.getHeading().getDegrees() + 180);
    }
}
