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

public class RetractArm extends CommandBase {
  private Climber climber;
  private Drivetrain drivetrain;
  private int RAISE = 0, CLIMB = 1;

  private int timer;
  private int timeLimit = 50;

  private boolean retract = false;  
  private boolean extendedAtStart = false;

  public RetractArm(Climber climber, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    extendedAtStart = climber.isExtended();
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("extended at start?", extendedAtStart);
    

    if (extendedAtStart) {
      if (climber.getEncoder(RAISE) > Constants.ClimberContants.CLIMBER_ARM_BOTTOM_THRESHOLD){
          climber.lowerClimber(0.3);
      } else {
          climber.stopClimber();
          timer++;

          if(timer > timeLimit){
            climber.retractArm();
            drivetrain.setSlowMode(false);
          }
          
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
    timer = 0;

    extendedAtStart = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
