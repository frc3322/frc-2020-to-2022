/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Robot.m_can;

public class Drivetrain extends SubsystemBase {
    private DifferentialDrive robotDrive;

    private CANSparkMax[] motors = new CANSparkMax[4];
    private RelativeEncoder[] encoders = new RelativeEncoder[4];

    private AHRS navx = new AHRS(SPI.Port.kMXP);

    private PIDController PID1;
    private PIDController PID2;

    private DifferentialDriveOdometry odometry;

    private final int LEFT_BACK = 0, LEFT_FRONT = 1, RIGHT_BACK = 2, RIGHT_FRONT = 3;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tv = table.getEntry("tv");

    private boolean slowMode = false;

    private boolean notUseLimelight = false;

    private double limelightX = tx.getDouble(0.0);
    private double limelightY = ty.getDouble(0.0);
    private double limelightTarget = tv.getDouble(0.0);

    private double lP = 0.045;
    private double lI = 0;
    private double lD = 0;

    private double aP = -0.0032;
    private double aI = 0;
    private double aD = 0;

    private double dP = 0;
    private double dI = 0;
    private double dD = 0;

    private double sP = 0.001;
    private double sI =  0;
    private double sD = 0;

    private double threshold = 2;

    public enum PIDMode {
        LIMELIGHT, ANGLE, DISTANCE, STRAIGHTEN, 
    }

    public Drivetrain() {
        motors[LEFT_BACK] = new CANSparkMax(m_can.LEFT_BACK_MOTOR, MotorType.kBrushless);
        motors[LEFT_FRONT] = new CANSparkMax(m_can.LEFT_FRONT_MOTOR, MotorType.kBrushless);
        motors[RIGHT_BACK] = new CANSparkMax(m_can.RIGHT_BACK_MOTOR, MotorType.kBrushless);
        motors[RIGHT_FRONT] = new CANSparkMax(m_can.RIGHT_FRONT_MOTOR, MotorType.kBrushless);

        motors[LEFT_BACK].restoreFactoryDefaults();
        motors[LEFT_FRONT].restoreFactoryDefaults();
        motors[RIGHT_BACK].restoreFactoryDefaults();
        motors[RIGHT_FRONT].restoreFactoryDefaults();

        motors[LEFT_FRONT].setInverted(true);
        motors[RIGHT_FRONT].setInverted(true);

        motors[LEFT_BACK].follow(motors[LEFT_FRONT]);
        motors[RIGHT_BACK].follow(motors[RIGHT_FRONT]);

        motors[LEFT_FRONT].setIdleMode(IdleMode.kBrake);
        motors[LEFT_BACK].setIdleMode(IdleMode.kBrake);
        motors[RIGHT_FRONT].setIdleMode(IdleMode.kBrake);
        motors[RIGHT_BACK].setIdleMode(IdleMode.kBrake);

        motors[LEFT_FRONT].setSmartCurrentLimit(70, 60);
        motors[LEFT_BACK].setSmartCurrentLimit(70, 60);
        motors[RIGHT_FRONT].setSmartCurrentLimit(70, 60);
        motors[RIGHT_BACK].setSmartCurrentLimit(70, 60);

        motors[LEFT_FRONT].burnFlash();
        motors[LEFT_BACK].burnFlash();
        motors[RIGHT_FRONT].burnFlash();
        motors[RIGHT_BACK].burnFlash();

        encoders[LEFT_BACK] = motors[LEFT_BACK].getEncoder();
        encoders[LEFT_FRONT] = motors[LEFT_FRONT].getEncoder();
        encoders[RIGHT_BACK] = motors[RIGHT_BACK].getEncoder();
        encoders[RIGHT_FRONT] = motors[RIGHT_FRONT].getEncoder();

        robotDrive = new DifferentialDrive(motors[LEFT_FRONT], motors[RIGHT_FRONT]);

        PID1 = new PIDController(lP, lI, lD);
        PID1.disableContinuousInput();

        PID2 = new PIDController(sP, sI, sD);
        PID2.disableContinuousInput();

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    }

    public void initPos() {
    }

    public void putInitialDash(){
        SmartDashboard.putNumber("Drivetrain/DrivePID/Limelight/Limelight P", lP);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Limelight/Limelight I", lI);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Limelight/Limelight D", lD);

        SmartDashboard.putNumber("Drivetrain/DrivePID/Angle/Angle P", aP);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Angle/Angle I", aI);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Angle/Angle D", aD);

        SmartDashboard.putNumber("Drivetrain/DrivePID/Distance/Distance P", dP);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Distance/Distance I", dI);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Distance/Distance D", dD);

        SmartDashboard.putNumber("Drivetrain/DrivePID/Distance/Straighten P", sP);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Distance/Straighten I", sI);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Distance/Strighten D", sD);

        SmartDashboard.putNumber("Drivetrain/DrivePID/Limelight/Setpoint", 0);
        SmartDashboard.putBoolean("Drivetrain/DrivePID/Using Limelight", false);
    }

    // Motor methods
    public double getVoltage(int n) {
        return motors[n].getBusVoltage();
    }

    public double getMotorHeat(int n) {
        return motors[n].getMotorTemperature();
    }

    public double getOutputCurrent(int n) {
        return motors[n].getOutputCurrent();
    }

    public double getEncoder(int n) {
        return encoders[n].getPosition();
    }

    public double getVelocity(int n) {
        return encoders[n].getVelocity();
    }

    // Driving methods
    public void drive(double speed, double rotation) {
        robotDrive.arcadeDrive(speed, rotation);
    }

    public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
        robotDrive.curvatureDrive(speed, rotation, quickTurn);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        robotDrive.tankDrive(leftSpeed, rightSpeed);
    }

    public void setSlowMode(boolean mode){
        slowMode = mode;
    }
    public boolean getSlowMode(){
        return slowMode;
    }

    // PID Control Methods
    public void setUpPID(PIDMode mode) {
        switch (mode) {
            case LIMELIGHT:
                PID1.reset();
                PID1.disableContinuousInput();
                PID1.setP(SmartDashboard.getNumber("Drivetrain/DrivePID/Limelight/Limelight P", lP));
                PID1.setI(SmartDashboard.getNumber("Drivetrain/DrivePID/Limelight/Limelight I", lI));
                PID1.setD(SmartDashboard.getNumber("Drivetrain/DrivePID/Limelight/Limelight D", lD));
                PID1.setTolerance(1);

                break;
            case ANGLE:
                PID1.reset();
                PID1.enableContinuousInput(-180.0, 180.0);
                PID1.setP(SmartDashboard.getNumber("Drivetrain/DrivePID/Angle/Angle P", aP));
                PID1.setI(SmartDashboard.getNumber("Drivetrain/DrivePID/Angle/Angle I", aI));
                PID1.setD(SmartDashboard.getNumber("Drivetrain/DrivePID/Angle/Angle D", aD));
                PID1.setTolerance(1);
                break;
            case DISTANCE:
                PID1.reset();
                PID1.disableContinuousInput();
                PID1.setP(SmartDashboard.getNumber("Drivetrain/DrivePID/Distance/Distance P", dP));
                PID1.setI(SmartDashboard.getNumber("Drivetrain/DrivePID/Distance/Distance I", dI));
                PID1.setD(SmartDashboard.getNumber("Drivetrain/DrivePID/Distance/Distance D", dD));
                PID1.setTolerance(1);
                break;
            case STRAIGHTEN:
                PID2.reset();
                PID2.disableContinuousInput();
                PID2.setP(SmartDashboard.getNumber("Drivetrain/DrivePID/Distance/Straighten P", sP));
                PID2.setI(SmartDashboard.getNumber("Drivetrain/DrivePID/Distance/Straighten I", sI));
                PID2.setD(SmartDashboard.getNumber("Drivetrain/DrivePID/Distance/Straighten D", sD));
                PID2.setTolerance(1);
                break;
            
            default:
                PID1.reset();
                PID1.disableContinuousInput();
                PID1.setP(SmartDashboard.getNumber("Drivetrain/DrivePID/Limelight/Limelight P", lP));
                PID1.setI(SmartDashboard.getNumber("Drivetrain/DrivePID/Limelight/Limelight I", lI));
                PID1.setD(SmartDashboard.getNumber("Drivetrain/DrivePID/Limelight/Limelight D", lD));
                PID1.setTolerance(1);
                break;
        }
    }

    // Limelight PID methods
    public void setLimelight(boolean on){
        if (on) {
            table.getEntry("ledMode").setNumber(3);
        } else {
            table.getEntry("ledMode").setNumber(1);
        }
        
    }

    public double getLimelightX() {
        limelightX = tx.getDouble(0.0);
        SmartDashboard.putNumber("Drivetrain/Limelight/Limelight tx", limelightX);
        return limelightX;
    }

    public double getLimelightY() {
        limelightY = ty.getDouble(0.0);
        SmartDashboard.putNumber("Drivetrain/Limelight/Limelight ty", limelightY);
        return limelightY;
    }

    public double getLimelightTarget() {
        double limelightTarget = tv.getDouble(0.0);
        return limelightTarget;
    }


    public void alime(Double initAngle, Double initTX) {
        notUseLimelight = false;
        SmartDashboard.putNumber("Drivetrain/Limelight/Limelight tx", limelightX);
        double setpoint = (initAngle - initTX);
        SmartDashboard.putNumber("Drivetrain/DrivePID/Limelight/Setpoint", setpoint);
        double PIDOutput = 0;
        SmartDashboard.putNumber("Drivetrain/DrivePID/Limelight/PIDOutput", PIDOutput);

        if (Math.abs(getLimelightX()) < 4) {
            notUseLimelight = true;
        }

        if (notUseLimelight) {
            SmartDashboard.putBoolean("Drivetrain/DrivePID/Using Limelight", true);
            drive(0, Math.copySign(0.31, getLimelightX()));
        } else {
            PIDOutput = PID1.calculate(getHeading(), setpoint);
            SmartDashboard.putBoolean("Drivetrain/DrivePID/Using Limelight", false);
            drive(0, -PIDOutput);
        }
    }

    public boolean alimeOnTarget() {
        if (Math.abs(limelightX) < 1) {
            threshold = 2;
        } else if(Math.abs(limelightX) > 2) {
            threshold = 1;
        }

        return Math.abs(getLimelightX()) < threshold;
    }

    // Other PID Drive methods
    // for angle
    public void turnToAngle(double angle) {
        double PIDOutput = PID1.calculate(getHeading(), angle);
        double boost = Math.copySign(0.4, PIDOutput);
        drive(0, boost + PIDOutput);
        SmartDashboard.putNumber("Turn PID Output", PID1.calculate(getHeading(), angle));
    }

    public boolean angleOnTarget(double angle) {
        return Math.abs(Math.abs(angle) - Math.abs(getHeading())) < 1;
    }

    public void resetNavX() {
        navx.reset();
    }

    // for distance
    public void driveDistance(double distance, double initAngle) {
        double avgDist = getLeftEncDistance() + getRightEncDistance() / 2;
        drive(PID1.calculate(avgDist, distance), PID2.calculate(getHeading(), initAngle));
    }

    //for going straight
    public void driveStraight(double speed, double initAngle) {
        drive(speed, PID2.calculate(getHeading(), initAngle));
    }

    public Boolean distanceOnTarget(double distance) {
        double avgDist = (getLeftEncDistance() + getRightEncDistance()) / 2;
        return Math.abs(Math.abs(distance) - Math.abs(avgDist)) < 0.1;
    }

    public void resetEncoders() {
        encoders[LEFT_FRONT].setPosition(0);
        encoders[RIGHT_FRONT].setPosition(0);
    }

    // to make a run command for delay
    public void delay() {
        //nothing
    }

    // Methods for Pathfinding Auton
    public void reset() {
        encoders[LEFT_FRONT].setPosition(0);
        encoders[RIGHT_FRONT].setPosition(0);
        encoders[LEFT_BACK].setPosition(0);
        encoders[RIGHT_BACK].setPosition(0);
        resetNavX();

        Rotation2d originRotation = new Rotation2d(0.0);
        Pose2d originPose = new Pose2d(0.0, 0.0, originRotation);
        odometry.resetPosition(originPose, originRotation);
    }

    // returns meters traveled
    public double getLeftEncDistance() {
        return encoders[LEFT_FRONT].getPosition() * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS
                * Constants.DriveConstants.GEARING;
    }

    public double getRightEncDistance() {
        return -encoders[RIGHT_FRONT].getPosition() * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS
                * Constants.DriveConstants.GEARING;
    }

    // returns meters per second
    public double getLeftEncRate() {
        double RPS = (encoders[LEFT_FRONT].getVelocity() * Constants.DriveConstants.GEARING) / 60;
        return RPS * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getRightEncRate() {
        double RPS = (encoders[RIGHT_FRONT].getVelocity() * Constants.DriveConstants.GEARING) / 60;
        return RPS * Constants.DriveConstants.WHEEL_CIRCUMFERENCE_METERS;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncRate(), getRightEncRate());
    }

    public double getHeading() {
        return -1 * Math.IEEEremainder(navx.getAngle(), 360) * (Constants.DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        robotDrive.feed();
        motors[LEFT_FRONT].setVoltage(leftVolts);
        motors[RIGHT_FRONT].setVoltage(-rightVolts);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drivetrain/Encoders/ENC Distance Left m", getLeftEncDistance());
        SmartDashboard.putNumber("Drivetrain/Encoders/ENC Distance Right m", getRightEncDistance());
        SmartDashboard.putNumber("Drivetrain/Encoders/Avg Distance", (getLeftEncDistance() + getRightEncDistance()) / 2);
        SmartDashboard.putNumber("Drivetrain/NavX", getHeading());
        //SmartDashboard.putBoolean("Drivetrain/OnTarget", distanceOnTarget(1));
        // SmartDashboard.putString("pose", getPose().toString());
        // Translation2d driveTranslation = getPose().getTranslation();
        // double driveX = driveTranslation.getX();
        // double driveY = driveTranslation.getY();
        // SmartDashboard.putNumber("PoseX", driveX);
        // SmartDashboard.putNumber("PoseY", driveX);

        SmartDashboard.putBoolean("Drivetrain/Limelight/Limelight on Target?", alimeOnTarget());
        getLimelightX();
        getLimelightY();
        // odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncDistance(),
        // getRightEncDistance());

        // var translation = odometry.getPoseMeters().getTranslation();

        // SmartDashboard.putNumber("Odometry X", translation.getX());
        // SmartDashboard.putNumber("Odometry Y", translation.getY());
    }
}
