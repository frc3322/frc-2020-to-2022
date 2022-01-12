package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import static frc.robot.Robot.m_can;

public class Feeder extends SubsystemBase {

    CANSparkMax[] motors = new CANSparkMax[2];

    private final int FEED_1 = 0, FEED_2 = 1;

    DigitalInput cellSensor;

    private boolean firstTime = true;
    private boolean shotSinceFed = false;
    private boolean intookSinceFed = false;
    private boolean cellSensorGot = false;
    private boolean timeout = false;
    private boolean autofeed = true;
    private Timer feederTimeoutTimer = new Timer();
    private double feederTimeout = 2.0;
    private boolean feederTimeoutStarted = false;
    private Timer feedTimer = new Timer();
    private double feedTimeLimit = 0.12;
    private boolean feederTimerStarted = false;

    public Feeder() {
        motors[FEED_1] = new CANSparkMax(m_can.FEEDER_1, MotorType.kBrushless);
        motors[FEED_2] = new CANSparkMax(m_can.FEEDER_2, MotorType.kBrushless);

        motors[FEED_1].restoreFactoryDefaults();
        motors[FEED_2].restoreFactoryDefaults();

        motors[FEED_1].setInverted(false);
        motors[FEED_2].setInverted(false);

        motors[FEED_1].setSmartCurrentLimit(20, 30);
        motors[FEED_2].setSmartCurrentLimit(25, 30);

        motors[FEED_1].setIdleMode(IdleMode.kBrake);
        motors[FEED_2].setIdleMode(IdleMode.kBrake);

        motors[FEED_1].burnFlash();
        motors[FEED_2].burnFlash();

        cellSensor = new DigitalInput(Constants.RobotMap.DIO.IR_ID);
    }

    public void initPos() {
        stop();
        setGotFalse();
    }
    
    public boolean getIR() {
            return cellSensor.get();
    }

    public void setGotFalse() {
        cellSensorGot = false;
    }

    public void feedTop(double speed) {
        motors[FEED_1].set(speed);
    }

    public void feedBottom(double speed) {
        motors[FEED_2].set(speed);
    }

    public void stop() {
        feedTop(0.0);
        feedBottom(0.0);
    }

    public void setShotSinceFed(boolean shot) {
        shotSinceFed = shot;
    }

    public void putInitialDash() {
        SmartDashboard.putNumber("Feed Speed Top", 0);
        SmartDashboard.putNumber("Feed Speed Bottom", 0);
        setGotFalse();
    }

    public void setTimeout(boolean m_timeout){
        timeout = m_timeout;
    }

    public void setAutofeed(boolean m_autofeed) {
        autofeed = m_autofeed;
    }

    public void updateDash() {
        
    }

    @Override
    public void periodic() {
        updateDash();
        feedTop(SmartDashboard.getNumber("Feeder/Feed Speed Top", 0));
        feedBottom(SmartDashboard.getNumber("Feeder/Feed Speed Bottom", 0));

        SmartDashboard.putBoolean("Feeder/AutoFeedTest/Intaking", RobotContainer.intaking);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/Shooting", RobotContainer.shooting);
        if(cellSensor.get() == true || cellSensor.get() == false){
            SmartDashboard.putBoolean("Feeder/AutoFeedTest/CellSensor", !cellSensor.get());
        }
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/IntookSinceFed", intookSinceFed);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/ShotSinceFed", shotSinceFed);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/CellSensorGot", cellSensorGot);
        SmartDashboard.putBoolean("Feeder/AutoFeedTest/FirstTime", firstTime);
        // SmartDashboard.putNumber("Feeder/AutoFeedTest/Timer", timer);
        // SmartDashboard.putNumber("Feeder/AutoFeedTest/TimeLimit", timeLimit);

        if (RobotContainer.intaking) {
            intookSinceFed = true;
        }

        if(timeout){
            if(!feederTimeoutStarted){
                feederTimeoutTimer.start();
                feederTimeoutStarted = true;
            }

            if(feederTimeoutTimer.get() > feederTimeout) {
                stop();
                timeout = false;
                autofeed = false;
                feederTimeoutStarted = false;
                feederTimeoutTimer.reset();
            }
        }
      
        if (intookSinceFed && autofeed) {
            if (firstTime || shotSinceFed) {
                feedTop(0.8);
                feedBottom(.8);
                
                if (!cellSensor.get()) {
                    cellSensorGot = true;
                }
                
                if (cellSensorGot) {
                    if(!feederTimerStarted){
                        feedTimer.start();
                        feederTimerStarted = true;
                    }
                    if (feedTimer.get() > feedTimeLimit) {
                        stop();
                        firstTime = false;
                        intookSinceFed = false;
                        shotSinceFed = false;
                        cellSensorGot = false;
                        feederTimerStarted = false;
                        feedTimer.reset();
                    }
                }
            }
        }
    }
}