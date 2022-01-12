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
import frc.robot.Constants;
import frc.robot.LedData;
import frc.robot.Constants.RobotMap;

import static frc.robot.Robot.m_can;

public class Climber extends SubsystemBase {
    private int RAISE = 0, CLIMB = 1;
    private CANSparkMax[] motors = new CANSparkMax[2];
    private RelativeEncoder[] encoders = new RelativeEncoder[2];
    DoubleSolenoid armExtender;

    public Climber() {
        motors[RAISE] = new CANSparkMax(m_can.CLIMBER_RAISE, MotorType.kBrushless);
        motors[CLIMB] = new CANSparkMax(m_can.CLIMBER_CLIMB, MotorType.kBrushless);
        
        motors[RAISE].restoreFactoryDefaults();
        motors[CLIMB].restoreFactoryDefaults();

        motors[RAISE].setInverted(false);
        motors[CLIMB].setInverted(true);

        motors[RAISE].setSmartCurrentLimit(40, 30);
        motors[CLIMB].setSmartCurrentLimit(80);
        motors[CLIMB].setSecondaryCurrentLimit(75);

        motors[RAISE].setIdleMode(IdleMode.kBrake);
        motors[CLIMB].setIdleMode(IdleMode.kBrake);
        
        motors[RAISE].burnFlash();
        motors[CLIMB].burnFlash();

        encoders[RAISE] = motors[RAISE].getEncoder();
        encoders[CLIMB] = motors[CLIMB].getEncoder();

        armExtender = new DoubleSolenoid(RobotMap.PCM.PCM_ID, PneumaticsModuleType.CTREPCM, Constants.RobotMap.PCM.ARM_EXTEND, Constants.RobotMap.PCM.ARM_RETRACT);
    }

    public boolean atBottom() {
        return encoders[RAISE].getPosition() < Constants.ClimberContants.CLIMBER_ARM_BOTTOM_THRESHOLD;
    }

    public void resetEncoders(){
        encoders[RAISE].setPosition(0.0);
    }

    public void raiseClimber(double speed) {
        motors[RAISE].set(speed);
    }

    public void lowerClimber(double speed) {
        motors[RAISE].set(-speed);
    }

    public void stopClimber() {
        motors[RAISE].set(0);
    }

    public void pullWinch(double speed) {
        LedData.getInstance().startPattern(LedData.LedMode.CLIMB);
        motors[CLIMB].set(speed);
    }

    public void pushWinch(double speed) {
        motors[CLIMB].set(-speed);
    }

    public void setWinch(double speed) {
        motors[CLIMB].set(speed);
    }

    public void stopWinch() {
        motors[CLIMB].set(0);
    }

    public void extendArm() {
        armExtender.set(DoubleSolenoid.Value.kForward);
    }

    public void retractArm() {
        armExtender.set(DoubleSolenoid.Value.kReverse);
    }

    public void toggle() {
        if (isExtended()) {
            retractArm();
        } else {
            extendArm();
        }
    }

    public double getEncoder(int encoder) {
        return encoders[encoder].getPosition();
    }

    public void setSpeed(double speed) {
        motors[RAISE].set(speed);
    }

    public boolean isExtended() {
        return armExtender.get() == Value.kForward;
    }

    public void putInitialDash() {
        SmartDashboard.putNumber("Climb Speed", 0);
    }

    public void initPos() {
        retractArm();
        resetEncoders();
    }

    @Override
    public void periodic() {
        //setSpeed(SmartDashboard.getNumber("Climb Speed", 0));
        SmartDashboard.putNumber("Climber/Climb Encoder", encoders[RAISE].getPosition());
        //SmartDashboard.putNumber("Voltage Draw Climber", motors[RAISE].getBusVoltage());
        SmartDashboard.putNumber("Climber/Output Current Climber", motors[RAISE].getOutputCurrent());
        SmartDashboard.putBoolean("Climber/At Bottom", atBottom());
    }
}
