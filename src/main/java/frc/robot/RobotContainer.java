/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.DriveControl;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.RetractArm;
import frc.robot.commands.Shoot;
import frc.robot.commands.auton.DriveDistance;
import frc.robot.commands.auton.DriveDistanceJank;
import frc.robot.commands.auton.TurnToAngle;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Drivetrain.PIDMode;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.RobotMap;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    private final Drivetrain drivetrain = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    private final Intake intake = new Intake();
    private final Feeder feeder = new Feeder();
    private final Climber climber = new Climber();
    
    private final Joystick lowerChassis = new Joystick(0);
    private final Joystick upperChassis = new Joystick(1);

    public enum auton {
        DEFAULT,
        SIX_BALL,
        MIDDLE_SIX;
    }

    //commands
    private Command shoot = new Shoot(drivetrain, shooter, feeder, hopper, intake, true);
    private Command shootWithoutAlime = new Shoot(drivetrain, shooter, feeder, hopper, intake, false);
    private Command timeoutShoot = new Shoot(drivetrain, shooter, feeder, hopper, intake, false);
    private Command cycleHopper = new RunCommand(() -> hopper.cycle(-0.5, -0.5));
    private Command extendArm = new ExtendArm(climber, drivetrain);
    private Command retractArm = new RetractArm(climber, drivetrain);
    private Command driveControl = new DriveControl(drivetrain, lowerChassis);
    

    //auton commands
    private Command defaultAuton;

    private Command feed = new InstantCommand(() -> intake.extend())
                            .andThen(new RunCommand(() -> intake.outtake()).withTimeout(3))
                            .andThen(new InstantCommand(() -> intake.retract()))
                            .andThen(new InstantCommand(() -> intake.stop()));

    private Command middleSix = new TurnToAngle(drivetrain, -25)
                                .andThen(new Shoot(drivetrain, shooter, feeder, hopper, intake, true).withTimeout(4.0))
                                .andThen(new TurnToAngle(drivetrain, -25))
                                .andThen(new DriveDistanceJank(drivetrain, -.7, -2.2))
                                .andThen(new InstantCommand(() -> intake.begin()))
                                .andThen(new TurnToAngle(drivetrain, -85))
                                .andThen(new DriveDistanceJank(drivetrain, .35, 1.25))
                                .andThen(new RunCommand(() -> drivetrain.drive(-0.35, -0.35)).withTimeout(1))
                                .andThen(new InstantCommand(() -> intake.end()))
                                .andThen(new TurnToAngle(drivetrain, 0))
                                .andThen(new DriveDistanceJank(drivetrain, 1, 1.5))
                                .andThen(new Shoot(drivetrain, shooter, feeder, hopper, intake, true));
    
    
    
                                // new Shoot(drivetrain, shooter, feeder, hopper, intake, true).withTimeout(4.0)
                                // .andThen(new TurnToAngle(drivetrain, 180))
                                // .andThen(new InstantCommand(() -> intake.begin()))
                                // .andThen(new DriveDistanceJank(drivetrain, .7, 1.7))
                                // .andThen(new RunCommand(() -> drivetrain.drive(0.5, 0.5)).withTimeout(2.2))
                                // .andThen(new RunCommand(() -> drivetrain.drive(-0.5, -0.5)).withTimeout(2.2))
                                // .andThen(new InstantCommand(() -> intake.end()))
                                // .andThen(new TurnToAngle(drivetrain, -30))
                                // .andThen(new Shoot(drivetrain, shooter, feeder, hopper, intake, true));

    private Command sixBall = new Shoot(drivetrain, shooter, feeder, hopper, intake, true).withTimeout(4.0)
                            .andThen(new TurnToAngle(drivetrain, 180))
                            .andThen(new DriveDistanceJank(drivetrain, 0.7, 1.6))
                            .andThen(new InstantCommand(() -> intake.begin()))
                            .andThen(new DriveDistanceJank(drivetrain, 0.6, 2.3))
                            .andThen(new RunCommand(() -> drivetrain.drive(-1, -0.7)).withTimeout(1))
                            .andThen(new InstantCommand(() -> intake.end()))
                            .andThen(new TurnToAngle(drivetrain, -1))
                            .andThen(new Shoot(drivetrain, shooter, feeder, hopper, intake, true));
    //test commands
    private Command testDriveDistance = new DriveDistanceJank(drivetrain, .4, 2.0);
    private Command testTurnToAngle = new TurnToAngle(drivetrain, 180); 
    private Command testTurnToAngleZero = new TurnToAngle(drivetrain, 0); 

    public static boolean intaking = false;
    public static boolean shooting = false;
    
    public RobotContainer() {
        
        configureButtonBindings();

        drivetrain.putInitialDash();
        feeder.putInitialDash();
        shooter.putInitialDash();
        hopper.putInitialDash();
        
        LedData.getInstance().setAlliance();
        climber.putInitialDash();

        drivetrain.setDefaultCommand(driveControl);
    }

    private void configureButtonBindings() {
        Button button_a_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_A);
        Button button_b_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_B);
        Button button_x_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_X);
        Button button_y_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_Y);
        Button bumper_left_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUMPER_LEFT);
        Button bumper_right_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUMPER_RIGHT);
        Button button_back_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_BACK);
        Button button_start_upper = new JoystickButton(upperChassis, RobotMap.XBOX.BUTTON_START);
        DPadButton dpad_down_upper = new DPadButton(upperChassis, DPadButton.Direction.DOWN);

        Button bumper_left_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUMPER_LEFT);
        Button bumper_right_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUMPER_RIGHT);
        Button button_back_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_BACK);
        Button button_start_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_START);
        Button left_stick_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.STICK_LEFT);
        Button right_stick_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.STICK_RIGHT);
        Button button_a_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_A);
        Button button_b_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_B);
        Button button_x_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_X);
        Button button_y_lower = new JoystickButton(lowerChassis, RobotMap.XBOX.BUTTON_Y);
        DPadButton dpad_up_lower = new DPadButton(lowerChassis, DPadButton.Direction.UP);
        DPadButton dpad_left_lower = new DPadButton(lowerChassis, DPadButton.Direction.LEFT);
        DPadButton dpad_right_lower = new DPadButton(lowerChassis, DPadButton.Direction.RIGHT);

        

        //upper
        button_a_upper.whenPressed(new InstantCommand(() -> shooter.setSetpoint(shooter.findRPM())))
                        .whenReleased(new InstantCommand(() -> shooter.stop()));

        button_b_upper.whenPressed(new InstantCommand(() -> testDriveDistance.schedule()))
                        .whenReleased(new InstantCommand(() -> testDriveDistance.cancel()));
                
        button_x_upper.whenPressed(new InstantCommand(() -> testTurnToAngleZero.schedule()))
                        .whenReleased(new InstantCommand(() -> testTurnToAngleZero.cancel()));
                        
        button_y_upper.whenPressed(new InstantCommand(() -> testTurnToAngle.schedule()))
                        .whenReleased(new InstantCommand(() -> testTurnToAngle.cancel()));

        bumper_left_upper.whenPressed(new InstantCommand(() -> shootWithoutAlime.schedule()))
                            .whenReleased(new InstantCommand(() -> shootWithoutAlime.cancel()));

        dpad_down_upper.whenPressed(new InstantCommand(() -> climber.pushWinch(0.3)))
                    .whenReleased(new InstantCommand(() -> climber.stopWinch()));

        // button_a_upper.whenPressed(new InstantCommand(() -> climber.raiseClimber(.3)))
        //                 .whenReleased(new InstantCommand(() -> climber.stopClimber()));

        // button_b_upper.whenPressed(new InstantCommand(() -> climber.lowerClimber(.3)))
        //                 .whenReleased(new InstantCommand(() -> climber.stopClimber()));

        // bumper_right_upper.whenPressed(new InstantCommand(() -> climber.toggle()));

        // button_y_upper.whenPressed(new InstantCommand(() -> climber.setWinch(.3)))
        //                 .whenReleased(new InstantCommand(() -> climber.stopWinch()));

        // button_x_upper.whenPressed(new InstantCommand(() -> climber.setWinch(-.3)))
        //                 .whenReleased(new InstantCommand(() -> climber.stopWinch()));
        //lower
        bumper_right_lower.whenPressed(new InstantCommand(() -> feeder.setTimeout(false))
                            .andThen(new InstantCommand(() -> feeder.setAutofeed(true)))
                            .andThen(new InstantCommand(() -> intake.begin()))
                            .andThen(new InstantCommand(() -> hopper.cycle(-0.3, 0.3))))
                            .whenReleased(new InstantCommand(() -> feeder.setTimeout(true))
                            .andThen(new InstantCommand(() -> intake.end()))
                            .andThen(new InstantCommand(() -> hopper.cycle(0, 0))));

        bumper_left_lower.whenPressed(new InstantCommand(() -> shootWithoutAlime.schedule()))
                            .whenReleased(new InstantCommand(() -> shootWithoutAlime.cancel()));

        button_a_lower.whenPressed(new InstantCommand(() -> shoot.schedule()))
                        .whenReleased(new InstantCommand(() -> shoot.cancel())
                        .alongWith(new InstantCommand(() -> hopper.stop())));

        button_b_lower.whenPressed(new InstantCommand(() -> retractArm.schedule()))
                        .whenReleased(new InstantCommand(() -> retractArm.cancel()));

        button_x_lower.whenPressed(new InstantCommand(() -> intake.outtakeBegin()))
                        .whenReleased(new InstantCommand(() -> intake.end()));

        button_y_lower.whenPressed(new InstantCommand(() -> extendArm.schedule()))
                        .whenReleased(new InstantCommand(() -> extendArm.cancel()));

        dpad_up_lower.whenPressed(new InstantCommand(() -> climber.pullWinch(1)))
                .whenReleased(new InstantCommand(() -> climber.stopWinch()));
        
        dpad_left_lower.whenPressed(new InstantCommand(() -> climber.lowerClimber(1)))
                        .whenReleased(new InstantCommand(() -> climber.stopClimber()));

        dpad_right_lower.whenPressed(new InstantCommand(() -> climber.retractArm()));

        button_start_lower.whenPressed(new InstantCommand(() -> hopper.cycle(0.8, 0.8)))
                            .whenReleased(new InstantCommand(() -> hopper.stop()));
    }

    public void resetDrive() {
        drivetrain.reset();
    }

    public void setDriveControl() {
        driveControl.schedule();
    }

    public void cancelDriveControl() {
        driveControl.cancel();
    }

    public void setLimelight(boolean on) {
        drivetrain.setLimelight(on);
    }
    
    public void putInitialDashes() {
        drivetrain.putInitialDash();
        feeder.putInitialDash();
        shooter.putInitialDash();
        hopper.putInitialDash();
        climber.putInitialDash();
    }

    public void setInitPos() {
        climber.initPos();
        drivetrain.initPos();
        feeder.initPos();
        hopper.initPos();
        intake.initPos();
        shooter.initPos();
    }
    
    public Command getAutonomousCommand(auton selected) {
        // //Set up auton trajectory
        // DifferentialDriveVoltageConstraint autoVoltageConstraint =
        //     new DifferentialDriveVoltageConstraint(
        //         new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
        //                                     Constants.DriveConstants.kvVoltSecondsPerMeter,
        //                                     Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
        //                                     Constants.DriveConstants.kDriveKinematics,
        //                                     10);
    
        // TrajectoryConfig config =
        // new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //                         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                         .setKinematics(Constants.DriveConstants.kDriveKinematics)
        //                         .addConstraint(autoVoltageConstraint);
    
        // Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     List.of(
        //         new Translation2d(1, 1),
        //         new Translation2d(2, -1)
        //     ),
        //     new Pose2d(3, 0, new Rotation2d(0)),
        //     config
        // );
        
        // var transform = drivetrain.getPose().minus(testTrajectory.getInitialPose());
        // testTrajectory = testTrajectory.transformBy(transform);

        // RamseteController disabledRamsete = new RamseteController() {
        //     @Override
        //     public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
        //             double angularVelocityRefRadiansPerSecond) {
        //         return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        //     }
        // };

        // var feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter,
        // Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
        // //second element was:
        
        // var m_leftReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_reference");
        // var m_leftMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("left_measurement");
        // var m_rightReference = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_reference");
        // var m_rightMeasurement = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("right_measurement");
        // //new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta)
        
        // RamseteCommand ramseteCommand = new RamseteCommand(testTrajectory, drivetrain::getPose,
        // disabledRamsete,
        // feedForward,
        // Constants.DriveConstants.kDriveKinematics, drivetrain::getWheelSpeeds,
        // new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0), new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
        // // RamseteCommand passes volts to the callback
        // (leftVolts, rightVolts) ->{
        //     drivetrain.tankDriveVolts(leftVolts, rightVolts);

        //     m_leftMeasurement.setNumber(feedForward.calculate(drivetrain.getWheelSpeeds().leftMetersPerSecond));
        //     m_leftReference.setNumber(leftVolts);

        //     m_rightMeasurement.setNumber(feedForward.calculate(drivetrain.getWheelSpeeds().rightMetersPerSecond));
        //     m_rightReference.setNumber(-rightVolts);
        // }, drivetrain);

        // Command trenchSixAuton = 
        //     timeoutShoot.withTimeout(3.0)
        //     .andThen(new TurnToAngle(drivetrain, 180.0))
        //     .andThen(new DriveDistance(drivetrain, 3.0))
        //         .alongWith(new InstantCommand(() -> intake.begin()))
        //     .andThen(new TurnToAngle(drivetrain, 180.0))
        //         .alongWith(new InstantCommand(() -> intake.end()))
        //     .andThen(timeoutShoot.withTimeout(4.0));

        defaultAuton = new RunCommand(() -> drivetrain.delay()).withTimeout(SmartDashboard.getNumber("Auton Delay", 0))
                        .andThen(timeoutShoot.withTimeout(4))
                        .andThen(new RunCommand(() -> drivetrain.drive(-0.5, 0.0)).withTimeout(1))
                        .andThen(new InstantCommand(() -> drivetrain.drive(0, 0)));

        switch (selected) {
            case MIDDLE_SIX:
                return middleSix;
            case SIX_BALL:
                return  sixBall;
            case DEFAULT:
                return defaultAuton; 
            default:
                return defaultAuton;
        }

        //return new InstantCommand(() -> drivetrain.delay());

        // Run path following command, then stop at the end.
        //return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    }
}
