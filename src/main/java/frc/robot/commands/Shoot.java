/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain.PIDMode;
import frc.robot.subsystems.Shooter.ShooterPIDMode;
import frc.robot.LedData;

public class Shoot extends CommandBase {
    Drivetrain drivetrain;
    Shooter shooter;
    Feeder feeder;
    Hopper hopper;
    Intake intake;
    boolean alime;
    double initAngle;
    double initTX;
    double angleSetpoint;

    Timer limeTimer = new Timer();
    double limeTimeLimit = 0.2; //seconds

    Timer shootTimer = new Timer();
    double shootTimeLimit = 0.2;   
    
    Timer lightDelayTimer = new Timer();
    double lightDelay = 1.0;

    double shootSetpoint;

    boolean limelightAligned = false;
    boolean shooterSped = false;
    boolean startLimeTimer = false;
    boolean shootPIDSetUp = false;
    boolean ledsSet = false;
    boolean lightOn = false;
    boolean feed = false;

    public Shoot(Drivetrain drivetrain, Shooter shooter, Feeder feeder, Hopper hopper, Intake intake, boolean alime) {
        this.drivetrain = drivetrain;
        addRequirements(shooter);

        this.shooter = shooter;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.alime = alime;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setUpPID(ShooterPIDMode.RAMP);
        if (alime) {
            LedData.getInstance().startPattern(LedData.LedMode.TARGET);
            //setpoint = SmartDashboard.getNumber("Shooter/ShootPID/Shooter Setpoint", 3000);
            drivetrain.setUpPID(PIDMode.LIMELIGHT);
            drivetrain.setLimelight(true);
        } else {
            drivetrain.setLimelight(true);
            //shootSetpoint = 3200;
        }

        lightDelayTimer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("limelightTargeting?", drivetrain.getLimelightTarget());
        if (!feed) {
            if(!lightOn) {
                //if(lightDelayTimer.get() > lightDelay){

                if(drivetrain.getLimelightTarget() == 1.0){
                    initAngle = drivetrain.getHeading();
                    initTX = drivetrain.getLimelightX();
                    angleSetpoint = initAngle - initTX;
                    shootSetpoint = shooter.findRPM();
                    
                    shooter.setSetpoint(shootSetpoint);
                    
                    lightOn = true;
                }

            } else {
                if (alime) {
                    SmartDashboard.putBoolean("Shooter/ShootPID/Shooter On Target", shooter.onTarget(shootSetpoint));
                    SmartDashboard.putBoolean("Drivetrain/Limelight/Limelight on Target?", drivetrain.alimeOnTarget());
                    SmartDashboard.putBoolean("Aliming", true);

                    //Align with limelight
                    drivetrain.alime(initAngle, initTX);

                    //Check if robot is aligned to target for given time
                    if(drivetrain.alimeOnTarget()){
                        if(limeTimer.get() == 0.0){
                            limeTimer.start();
                        }
                        
                        if(limeTimer.get() > limeTimeLimit){
                            drivetrain.drive(0,0);
                            limelightAligned = true;
                        }
                    } else {
                        limeTimer.reset();
                    }
                    //Check if shooter is at speed for given time
                    if (shooter.onTarget(shootSetpoint)) {
                        if(shootTimer.get() == 0.0){
                            shootTimer.start();
                        }
                        
                        if(shootTimer.get() > shootTimeLimit){
                            shooterSped = true;
                        }

                    } else {
                        shootTimer.reset();
                    }

                    //Feed if both robot is aligned and shooter is at speed
                    if(limelightAligned && shooterSped){
                        feed = true;
                    }
                    
                } else {
                    SmartDashboard.putBoolean("Aliming", true);
                    //Check if shooter is at speed for given time
                    if (shooter.onTarget(shootSetpoint)) {
                        if(shootTimer.get() == 0.0){
                            shootTimer.start();
                        }
                        
                        if(shootTimer.get() > shootTimeLimit){
                            shooterSped = true;
                        }

                    } else {
                        shootTimer.reset();
                    }

                    if (shooterSped) {
                        feed = true;
                    }
                }
            }
        } else {
            if(!shootPIDSetUp){
                shooter.setUpPID(ShooterPIDMode.SHOOT);
                shootPIDSetUp = true;
            }
            
            if(!ledsSet){
                LedData.getInstance().startPattern(LedData.LedMode.SHOOT);
                drivetrain.setLimelight(false);
                ledsSet = true;
            }
                
            drivetrain.drive(0,0);
            feeder.feedTop(0.8);
            feeder.feedBottom(0.8);
            hopper.cycle(-0.3, -0.3);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LedData.getInstance().startPattern(LedData.LedMode.IDLE);
        feeder.stop();
        shooter.stop();
        limeTimer.reset();
        shootTimer.reset();
        feed = false;
        hopper.stop();
        feeder.setShotSinceFed(true);
        limelightAligned = false;
        shooterSped = false;
        startLimeTimer = false;
        shootPIDSetUp = false;
        ledsSet = false;
        lightOn = false;
        lightDelayTimer.reset();
        drivetrain.setLimelight(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
