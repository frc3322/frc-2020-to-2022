package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.PIDMode;

public class TurnToAngle extends CommandBase {

    Drivetrain drivetrain;

    private double angle;

    public TurnToAngle(Drivetrain drivetrain, double angle) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        this.angle = angle;
    }

    @Override
    public void initialize(){
        drivetrain.setUpPID(PIDMode.ANGLE);
    }

    @Override
    public void execute() {
        drivetrain.turnToAngle(angle);
    }

    @Override
    public void end(boolean interrupted){
        drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished(){
        return drivetrain.angleOnTarget(angle);
    }
}