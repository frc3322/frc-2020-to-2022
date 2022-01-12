/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum whichBot {
        TEST_BENCH,
        P1,
        P2
    }
    
    public static class RobotMap {
        public static class CAN {

            public final int LEFT_FRONT_MOTOR;
            public final int LEFT_BACK_MOTOR;
            public final int RIGHT_FRONT_MOTOR;
            public final int RIGHT_BACK_MOTOR;

            public final int LEFT_HOPPER_MOTOR;
            public final int RIGHT_HOPPER_MOTOR;
    
            public final int INTAKE_BOTTOM;
            public final int INTAKE_TOP;

            public final int FEEDER_1;
            public final int FEEDER_2;

            public final int CLIMBER_RAISE;
            public final int CLIMBER_CLIMB;

            public final int SHOOTER_1;
            public final int SHOOTER_2;

            public CAN() {
                // TODO: switch the part of the enum to match robot currently being deployed to (TEST_BENCH, P1, or P2)
                switch(whichBot.P1){
                    case TEST_BENCH:

                        LEFT_FRONT_MOTOR = 8;
                        LEFT_BACK_MOTOR = 0;
                        RIGHT_FRONT_MOTOR = 36;
                        RIGHT_BACK_MOTOR = 0;
        
                        // TODO: give these actual IDs
                        FEEDER_1 = 0;
                        FEEDER_2 = 0;

                        CLIMBER_RAISE = 40;
                        CLIMBER_CLIMB = 0;

                        LEFT_HOPPER_MOTOR = 0;
                        RIGHT_HOPPER_MOTOR = 0;

                        INTAKE_BOTTOM = 0;
                        INTAKE_TOP = 0;
                    
                        SHOOTER_1 = 0;
                        SHOOTER_2 = 0;
                        break;
                    case P1:

                        LEFT_FRONT_MOTOR = 45;
                        LEFT_BACK_MOTOR = 4;
                        RIGHT_FRONT_MOTOR = 15;
                        RIGHT_BACK_MOTOR = 6;

                        FEEDER_1 = 9;
                        FEEDER_2 = 14;

                        CLIMBER_RAISE = 2;
                        CLIMBER_CLIMB = 36;
        
                        LEFT_HOPPER_MOTOR = 12;
                        RIGHT_HOPPER_MOTOR = 13;
            
                        INTAKE_BOTTOM = 20;
                        INTAKE_TOP = 40;
                    
                        SHOOTER_1 = 10;
                        SHOOTER_2 = 11;
                        break;
                    case P2:

                        // TODO: set these to be P2 CAN IDs
                        LEFT_FRONT_MOTOR = 34;
                        LEFT_BACK_MOTOR = 46;
                        RIGHT_FRONT_MOTOR = 37;
                        RIGHT_BACK_MOTOR = 35;

                        FEEDER_1 = 20;
                        FEEDER_2 = 21;

                        CLIMBER_RAISE = 0;
                        CLIMBER_CLIMB = 1;
        
                        LEFT_HOPPER_MOTOR = 0;
                        RIGHT_HOPPER_MOTOR = 0;
            
                        INTAKE_BOTTOM = 0;
                        INTAKE_TOP = 0;
                    
                        SHOOTER_1 = 0;
                        SHOOTER_2 = 1;
                        break;
                    default:
                        // default IDs are currently Test Bench ones.
                        LEFT_FRONT_MOTOR = 37;
                        LEFT_BACK_MOTOR = 46;
                        RIGHT_FRONT_MOTOR = 34;
                        RIGHT_BACK_MOTOR = 35;
    
                        // TODO: give these actual IDs
                        FEEDER_1 = 20;
                        FEEDER_2 = 21;

                        CLIMBER_RAISE = 0;
                        CLIMBER_CLIMB = 1;

                        LEFT_HOPPER_MOTOR = 0;
                        RIGHT_HOPPER_MOTOR = 0;
        
                        INTAKE_BOTTOM = 0;
                        INTAKE_TOP = 0;
                
                        SHOOTER_1 = 0;
                        SHOOTER_2 = 1;
                        break;
                }
            }
        }
    
        public static class DIO {
            public static final int IR_ID = 0;
        }

        // no real purpose
        public static class JOJO {

        }
    
        public static class PCM {
              
            //TODO: need to give these values 
            public static final int PCM_ID = 0;
            public static final int INTAKE_EXTEND = 2;
            public static final int INTAKE_RETRACT = 3;
            public static final int ARM_EXTEND = 1;
            public static final int ARM_RETRACT = 0;
            
        }
    
        public static class XBOX {
            // Buttons
            public static final int BUTTON_A = 1;
            public static final int BUTTON_B = 2;
            public static final int BUTTON_X = 3;
            public static final int BUTTON_Y = 4;
            public static final int BUMPER_LEFT = 5;
            public static final int BUMPER_RIGHT = 6;
            public static final int BUTTON_BACK = 7;
            public static final int BUTTON_START = 8;
            public static final int STICK_LEFT = 9;
            public static final int STICK_RIGHT = 10;
    
            // Axes
            public static final int STICK_L_X_AXIS = 0;
            public static final int STICK_L_Y_AXIS = 1;
            public static final int STICK_R_X_AXIS = 4;
            public static final int STICK_R_Y_AXIS = 5;
            public static final int TRIGGER_L_AXIS = 2;
            public static final int TRIGGER_R_AXIS = 3;

            public static final double MIN_DEADZONE = 0.13;
            public static final double MAX_DEADZONE = 0.15;
            public static final double SLOW_MODE_MULTIPLIER = 0.5;
        }
    }

    public static class DriveConstants {
        public static final double GEARING = 242.0/2480.0;
        public static final double WHEEL_DIAMETER_INCHES = 6;
        public static final double WHEEL_DIAMETER_METERS = WHEEL_DIAMETER_INCHES * 0.0254;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final boolean GYRO_REVERSED = false;
		public static final double ksVolts = 0.161; //0.161
		public static final double kvVoltSecondsPerMeter = 3; //3.05
        public static final double kaVoltSecondsSquaredPerMeter = 0.44; //0.475
        public static final double kTrackwidthMeters = 0.66; //0.66
		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
		public static final double kPDriveVel = 0; //0.4
    }

    public static class AutoConstants {
		public static final double kRamseteB = 2;
		public static final double kRamseteZeta = 0.7;
		public static double kMaxSpeedMetersPerSecond = 5;
		public static double kMaxAccelerationMetersPerSecondSquared = 2.5;
    }

    public static class ShooterConstants {
        public static final double SHOOTER_TOLERANCE = 25.0;
    }

    public static class ClimberContants {
        public static final double CLIMBER_ARM_TOP_LIMIT = 2000;
        public static final double CLIMBER_ARM_BOTTOM_THRESHOLD = 0.5;
        public static final double ARM_EXTEND_SPEED = 0.3;
        public static final double WINCH_EXTEND_SPEED = ARM_EXTEND_SPEED * -2;
    }
}
