/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer.auton;

public class Robot extends TimedRobot {
    
    private RobotContainer m_robotContainer;
    private static Command m_autonomousCommand;
    public static Constants.RobotMap.CAN m_can;
    private SendableChooser<auton> autonMode;
    
    @Override
    public void robotInit() {
        autonMode = new SendableChooser<>();

        autonMode.setDefaultOption("Default", auton.DEFAULT);
        autonMode.addOption("Six_Ball", auton.SIX_BALL);
        autonMode.addOption("Middle_Six", auton.MIDDLE_SIX);

        SmartDashboard.putData("Auton", autonMode);
        SmartDashboard.putNumber("Auton Delay", 0);

        m_can = new Constants.RobotMap.CAN();
        m_robotContainer = new RobotContainer();
        
        m_robotContainer.putInitialDashes();

        m_robotContainer.setInitPos();

        

    }

    @Override
    public void robotPeriodic() {
        //m_robotContainer.setLimelight(true);
        CommandScheduler.getInstance().run();
        
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.setInitPos();
        m_robotContainer.resetDrive();
        m_robotContainer.cancelDriveControl();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand(autonMode.getSelected());
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        SmartDashboard.putNumber("Auton Delay", 0);
        m_robotContainer.setInitPos();
        //m_robotContainer.setLimelight(false);
        SmartDashboard.putData("Auton", autonMode);
        //m_robotContainer.setDriveControl();
        m_robotContainer.resetDrive();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}
