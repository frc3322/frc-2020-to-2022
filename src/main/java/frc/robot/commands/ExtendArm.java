/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

public class ExtendArm extends CommandBase {
  private Climber climber;
  private Drivetrain drivetrain;
  private int RAISE = 0, CLIMB = 1;

  private int timer;
  private int timeLimit = 165;

  private boolean extend = false;  
  private boolean extendedAtStart = false;

  public ExtendArm(Climber climber, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    extendedAtStart = climber.isExtended();
    drivetrain.setSlowMode(true);
    climber.extendArm();
    extend = false;
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("extended at start?", extendedAtStart);
    if(!extendedAtStart && !extend){
      timer++;
      if(timer > timeLimit) {
        climber.resetEncoders();
        extend = true;
      }
    } else {
      extend = true;
    }

    if (extend) {
      if (climber.getEncoder(RAISE) < Constants.ClimberContants.CLIMBER_ARM_TOP_LIMIT){
          climber.raiseClimber(Constants.ClimberContants.ARM_EXTEND_SPEED);
      } else {
          climber.stopClimber();
          climber.stopWinch();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
    climber.stopWinch();
    timer = 0;
    extendedAtStart = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
