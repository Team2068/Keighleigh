// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

    public final static double ROBOT_HEIGHT = 0.0;//adjust

    public final static class ShooterConstants{
        public final static int FLYWHEEL_1 = 14;
        public final static int FLYWHEEL_2 = 15;
    }
   
    /**
     * 
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 21.5; // FIXME Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final int CURRENT_LIMIT = 30;
    public static final double NORMAL_SPEED = .5;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 19.5; // FIXME Measure and set wheelbase

    

    // public final static class GameElementConstants {
    //     public final static double UPPER_HUB = 2.64; //meters
    //     public final static double LOWER_HUB = 1.04; //meters
    //     public final static double HIGH_HEIGHT = UPPER_HUB - ROBOT_HEIGHT;
    //     public final static double LOW_HEIGHT = LOWER_HUB - ROBOT_HEIGHT;
    //   }
    // public final static class IntakeConstants{
    // public static final int CONVEYER_MOTOR_ONE = 10;
    // public static final int CONVERYER_TWO = 11;
    // public static final int INTAKE = 12;
    // public static final double Intakespeed = .2;
    // public static final double SPIT_OUT_BALL = -.2;
    // }
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(75.0); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(153); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(295); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(5); // FIXME Measure and set back right steer offset
    
    public final static class GameElementConstants {
        public final static double UPPER_HUB = 2.64; //meters
        public final static double LOWER_HUB = 1.04; //meters
        public final static double HIGH_HEIGHT = UPPER_HUB; //Subtract by robot height
        public final static double LOW_HEIGHT = LOWER_HUB;
    }
    
    public final static class AimbotConstants{
        public static final double baseSpeed = 0.5;
        public static final double minimumAdjustment = 1.1;
        public static final double Kp = 5;
        public static final double Ki = 0;
        public static final double Kd = 0.8;
    }

    public static final class AutoConstants {
        public static final double MAX_Speed_MetersPerSecond = 0.6;
        public static final double MAX_Acceleration_MetersPerSecondSquared = 0.4;
        public static final double MAX_AngularSpeed_RadiansPerSecond = Math.PI;
        public static final double Max_AngularSpeed_RadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(MAX_AngularSpeed_RadiansPerSecond, Max_AngularSpeed_RadiansPerSecondSquared);
    }

    public final static class LimelightConstants {
        public final static class LedMode {
            public final static int DEFAULT = 0;
            public final static int FORCE_OFF = 1;
            public final static int FORCE_BLINK = 2;
            public final static int FORCE_ON = 3;
        }
        public final static class CamMode {
            public final static int VISION = 0;
            public final static int DRIVER = 1;
        }
        public final static class StreamMode {
            public final static int STANDARD = 0;
            public final static int PIP_MAIN = 1;
            public final static int PIP_SECONDARY = 2;
        }
        public final static class SnapshotMode {
            public final static int NO_SNAPSHOT = 0;
            public final static int TWO_SNAPSHOTS = 1;
        }
        public final static class Pipelines {
            public final static int REFLECTIVE_TAPE = 0;
            public final static int RED_BALLS = 1;
            public final static int BLUE_BALLS = 2;
        }
        public final static double LIMELIGHT_HEIGHT = 29.5; // This is for testing, needs to be changed for actual robot
        public final static double LIMELIGHT_ANGLE = 0;
    }
}