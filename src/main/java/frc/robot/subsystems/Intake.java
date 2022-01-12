/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotMap;

import static frc.robot.Robot.m_can;

public class Intake extends SubsystemBase {
    /**
     * Creates a new Intake.
     */
    private CANSparkMax[] motors = new CANSparkMax[2];
    private RelativeEncoder[] encoders = new RelativeEncoder[2];
    DoubleSolenoid intakeExtender;

    private final int INTAKE_BOTTOM = 0, INTAKE_TOP = 1;

    public Intake() {
        intakeExtender = new DoubleSolenoid(RobotMap.PCM.PCM_ID, PneumaticsModuleType.CTREPCM, RobotMap.PCM.INTAKE_EXTEND,
                RobotMap.PCM.INTAKE_RETRACT);
        
        motors[INTAKE_BOTTOM] = new CANSparkMax(m_can.INTAKE_BOTTOM, MotorType.kBrushless);
        motors[INTAKE_TOP] = new CANSparkMax(m_can.INTAKE_TOP, MotorType.kBrushless);

        motors[INTAKE_BOTTOM].restoreFactoryDefaults();
        motors[INTAKE_TOP].restoreFactoryDefaults();

        motors[INTAKE_BOTTOM].setInverted(false);
        motors[INTAKE_TOP].setInverted(false);

        motors[INTAKE_BOTTOM].setIdleMode(IdleMode.kCoast);
        motors[INTAKE_TOP].setIdleMode(IdleMode.kCoast);

        motors[INTAKE_BOTTOM].setSmartCurrentLimit(25, 35);
        motors[INTAKE_TOP].setSmartCurrentLimit(60, 70);

        motors[INTAKE_BOTTOM].burnFlash();
        motors[INTAKE_TOP].burnFlash();

        encoders[INTAKE_BOTTOM] = motors[INTAKE_BOTTOM].getEncoder();
        encoders[INTAKE_TOP] = motors[INTAKE_TOP].getEncoder();


    }

    public void initPos() {
        end();
    }

    public void begin() {
        RobotContainer.intaking = true;
        start();
        extend();
    }

    public void end() {
        RobotContainer.intaking = false;
        stop();
        retract();
    }

    public void outtakeBegin() {
        outtake();
        extend();
    }

    public void start() {
        motors[INTAKE_BOTTOM].set(1);
        motors[INTAKE_TOP].set(-.8);
    }

    public void outtake() {
        motors[INTAKE_BOTTOM].set(-1);
        motors[INTAKE_TOP].set(1);
    }

    public void set(double topSpeed, double bottomSpeed) {
        motors[INTAKE_TOP].set(topSpeed);
        motors[INTAKE_BOTTOM].set(bottomSpeed);
    }

    public void stop() {
        motors[INTAKE_BOTTOM].set(0);
        motors[INTAKE_TOP].set(0);
    }

    public void extend() {
        intakeExtender.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        intakeExtender.set(DoubleSolenoid.Value.kReverse);
    }

    public double getVoltage(final int n) {
        return motors[n].getBusVoltage();
    }

    public double getMotorHeat(final int n) {
        return motors[n].getMotorTemperature();
    }

    public double getOutputCurrent(final int n) {
        return motors[n].getOutputCurrent();
    }

    public double getEncoder(final int n) {
        return encoders[n].getPosition();
    }

    public double getVelocity(final int n) {
        return encoders[n].getVelocity();
    }

    public void toggle() {
        if (isExtended()) {
            retract();
        } else {
            extend();
        }
    }

    public boolean isExtended() {
        return intakeExtender.get() == Value.kForward;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Draw Intake Bottom", motors[INTAKE_BOTTOM].getOutputCurrent());
        SmartDashboard.putNumber("Current Draw Intake Top", motors[INTAKE_TOP].getOutputCurrent());
    }
}
