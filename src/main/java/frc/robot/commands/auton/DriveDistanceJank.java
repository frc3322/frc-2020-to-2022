package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.PIDMode;

public class DriveDistanceJank extends CommandBase {

    Drivetrain drivetrain;

    private double speed;
    private double distance;
    private double initAngle;

    public DriveDistanceJank(Drivetrain drivetrain, double speed, double distance) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.speed = speed;
        this.distance = distance;
    }
    
    @Override
    public void initialize(){
        drivetrain.setUpPID(PIDMode.STRAIGHTEN);
        drivetrain.resetEncoders();

        initAngle = drivetrain.getHeading();
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Drivetrain/OnTarget", drivetrain.distanceOnTarget(distance));
        drivetrain.driveStraight(speed, initAngle);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished(){
        return drivetrain.distanceOnTarget(distance);
    }
}