/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

import static frc.robot.Robot.m_can;

public class Shooter extends SubsystemBase {
    
    private static double rP = 0.00035;
    private static double rI = 0.0000012;
    private static double rD = 0.001;
    private static double rF = 0;
    
    private static double sP = 0.00025;
    private static double sI = 0;
    private static double sD = 0;
    private static double sF = 0;
    
    public enum ShooterPIDMode {
        RAMP, SHOOT
    }

    private CANSparkMax[] motors = new CANSparkMax[2];
    private RelativeEncoder[] encoders = new RelativeEncoder[2];

    private final int MOTOR_0 = 0, MOTOR_1 = 1;

    private double[] distances = {10, 16, 20, 23, 25, 27};

    private double[] RPMs = {3250, 3350, 3400, 3700, 3800, 3900};

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry ty = table.getEntry("ty");

    private double limelightY = ty.getDouble(0.0);

    SparkMaxPIDController controller;

    public Shooter() {
        motors[MOTOR_0] = new CANSparkMax(m_can.SHOOTER_1, MotorType.kBrushless);
        motors[MOTOR_1] = new CANSparkMax(m_can.SHOOTER_2, MotorType.kBrushless);

        motors[MOTOR_0].restoreFactoryDefaults();
        motors[MOTOR_1].restoreFactoryDefaults();

        motors[MOTOR_0].setInverted(false);
        motors[MOTOR_1].setInverted(false);

        motors[MOTOR_0].setIdleMode(IdleMode.kCoast);
        motors[MOTOR_1].setIdleMode(IdleMode.kCoast);

        motors[MOTOR_0].setSmartCurrentLimit(50, 40);
        motors[MOTOR_1].setSmartCurrentLimit(50, 40);

        motors[MOTOR_1].follow(motors[MOTOR_0], true);
        
        motors[MOTOR_0].burnFlash();
        motors[MOTOR_1].burnFlash();

        encoders[MOTOR_0] = motors[MOTOR_1].getEncoder();
        encoders[MOTOR_1] = motors[MOTOR_1].getEncoder();

        controller = motors[MOTOR_0].getPIDController();
        controller.setOutputRange(0, 1);
    }

    public void setUpPID(ShooterPIDMode pidMode) {
        switch(pidMode){
            case RAMP:
                controller.setP(SmartDashboard.getNumber("Shooter/ShootPID/Shooter rP", rP));
                controller.setI(SmartDashboard.getNumber("Shooter/ShootPID/Shooter rI", rI));
                controller.setD(SmartDashboard.getNumber("Shooter/ShootPID/Shooter rD", rD));
                controller.setFF(SmartDashboard.getNumber("Shooter/ShootPID/Shooter rF", rF));
                break;
            case SHOOT:
                controller.setP(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sP", sP));
                controller.setI(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sI", sI));
                controller.setD(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sD", sD));
                controller.setFF(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sF", sF));
                break;
            default:
                controller.setP(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sP", rP));
                controller.setI(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sI", rI));
                controller.setD(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sD", rD));
                controller.setFF(SmartDashboard.getNumber("Shooter/ShootPID/Shooter sF", rF));
                break;
        }

    }

    public void initPos() {
        stop();
    }

    public void setSetpoint(double setpoint) {
        controller.setReference(setpoint, ControlType.kVelocity);
    }

    public double getDistance() {
        limelightY = ty.getDouble(0.0);
        double limelightAngle = 10;
        double targetAngle = limelightY;
        SmartDashboard.putNumber("Limelight/Target Angle", targetAngle);
        double limelightHeight = (2+(1/12));
        double targetHeight = (7 + (5/6));

        return ((targetHeight-limelightHeight)/(Math.tan((limelightAngle + targetAngle) * Math.PI/180)));
    }

    public double findRPM() {
        double myNumber = getDistance();
        double distance = Math.abs(distances[0] - myNumber);
        int idx = 0;
        for(int c = 1; c < distances.length; c++){
            double cdistance = Math.abs(distances[c] - myNumber);
            if(cdistance < distance){
                idx = c;
                distance = cdistance;
            }
        }

        if(idx < distances.length && idx < RPMs.length){
            return RPMs[idx];
        } else {
            return 0;
        }

        // return (39 * getDistance()) + 2810;
    }

    public double publishRPM() {
        SmartDashboard.putNumber("Shooter/Shooter RPM", encoders[MOTOR_0].getVelocity());
        return encoders[MOTOR_0].getVelocity();
    }

    public void setSpeed(double speed) {
        motors[MOTOR_0].set(speed);
    }

    public void stop(){
        motors[MOTOR_0].setVoltage(0);
    }

    public boolean onTarget(double setpoint) {
        return Math.abs(encoders[MOTOR_0].getVelocity() - setpoint) < Constants.ShooterConstants.SHOOTER_TOLERANCE;
    }

    public void putInitialDash(){
        SmartDashboard.putNumber("Shooter/Shooter Speed", 0);

        SmartDashboard.putNumber("Shooter/ShootPID/Shooter rP", rP);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter rI", rI);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter rD", rD); 
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter rF", rF);

        SmartDashboard.putNumber("Shooter/ShootPID/Shooter sP", sP);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter sI", sI);
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter sD", sD); 
        SmartDashboard.putNumber("Shooter/ShootPID/Shooter sF", sF);

        SmartDashboard.putNumber("Shooter/ShootPID/Shooter Setpoint", 3000);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM Setpoint", findRPM());
        SmartDashboard.putNumber("Shooter Distance", getDistance());
        publishRPM();
    }
}
