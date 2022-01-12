/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMap;

/**
 * An example command that uses an example subsystem.
 */
public class DriveControl extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Drivetrain drivetrain;
    private final Joystick lowerChassis;
    private double x;
    private double y;
    private double scaledX;
    private double outputTurn;
    private double a = -67551.4;
    private double b = 67549.5;
    private double c = 2.8809;
    private double pow1 = 1.66736;
    private double pow2 = 1.6674;
    private double direction = 1.0;
    private double deadzone = RobotMap.XBOX.MIN_DEADZONE;

    public DriveControl(Drivetrain subsystem, Joystick joystick) {
        drivetrain = subsystem;
        lowerChassis = joystick;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        x = lowerChassis.getRawAxis(RobotMap.XBOX.STICK_R_X_AXIS);
        y = -lowerChassis.getRawAxis(RobotMap.XBOX.STICK_L_Y_AXIS);

        // SmartDashboard.putNumber("X", x);
        // SmartDashboard.putNumber("Y", y);

        // double m_x = Math.abs(x);

        // if(x >= 0){
        //     direction = 1;
        // } else {
        //     direction = -1;
        // }

        // outputTurn = direction * ((a*Math.pow(m_x, pow1)) + (b*Math.pow(m_x, pow2) + (c * m_x)));

        // if(Math.abs(y) < 0.37){
        //     drivetrain.drive(y, outputTurn);
        // } else {
        //     drivetrain.drive(y, x);
        // }

        SmartDashboard.putNumber("Drivetrain/x", x);
        SmartDashboard.putNumber("Drivetrain/y", y);

        if(Math.abs(x) >= RobotMap.XBOX.MAX_DEADZONE){
            deadzone = RobotMap.XBOX.MIN_DEADZONE;
        } else if (Math.abs(x) <= RobotMap.XBOX.MIN_DEADZONE) {
            deadzone = RobotMap.XBOX.MAX_DEADZONE;
        }
        
        if(Math.abs(y) >= RobotMap.XBOX.MAX_DEADZONE){
            deadzone = RobotMap.XBOX.MIN_DEADZONE;
        } else if (Math.abs(y) <= RobotMap.XBOX.MIN_DEADZONE) {
            deadzone = RobotMap.XBOX.MAX_DEADZONE;
        }

        if (Math.abs(x) < deadzone) {
            x = 0.0;
        }

        if (Math.abs(y) < deadzone) {
            y = 0.0;
        }

        if(x < 0){
            scaledX = -((2 * Math.pow(-x, 3)) + (-2.2 * Math.pow(-x, 2)) + (1.2 * (-x)));
        } else {
            scaledX = ((2 * Math.pow(x, 3)) + (-2.2 * Math.pow(x, 2)) + (1.2 * (x)));
        }


        // double sign = 1;

        // Math.copySign(sign, x);

        // double scaledX = sign * ((2.6 * Math.pow(sign * x, 3)) - (2.6 * Math.pow(sign * x, 2)) + (sign * x));

        if(drivetrain.getSlowMode()){
            x *= RobotMap.XBOX.SLOW_MODE_MULTIPLIER;
            y *= RobotMap.XBOX.SLOW_MODE_MULTIPLIER;
        }

        if (y == 0.0) {
            drivetrain.curvatureDrive(0, scaledX, true);
        } else {
            drivetrain.curvatureDrive(y, x, false);
        }
        
    }
}
