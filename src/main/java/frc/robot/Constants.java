// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    //PID constants

    public static final double DRIVE_STATIC_GAIN = .5;
    public static final double DRIVE_VELOCITY_GAINS = .5;
    public static final double DRIVE_MAX_ACCELERATION_GAINS = .5;
    public final static double ROBOT_HEIGHT = 0.0;// adjust

    public final static class ShooterConstants {
        public final static int FLYWHEEL_1 = 11;
        public final static int FLYWHEEL_2 = 12;
        public final static double kS = 0.13243;
        public final static double kV = 0.12573;
        public final static double kA = 0.0058814;
        public final static double kP = 0.00018; // OLD: 0.10976
        public final static double LOWER_HUB_RPM = 1800;
        public final static double UPPER_HUB_FALLBACK_RPM = 3900; // use this if the limelight doesnt work
        public final static double[] distTable = new double[5];
        public final static double[] rpmTable = new double[5];
        public final static void configureTable(){
            distTable[0] = 220;
            rpmTable[0] = 3400;

            distTable[1] = 300;
            rpmTable[1] = 3700;

            distTable[2] = 380;
            rpmTable[2] = 3900;

            distTable[3] = 464;
            rpmTable[3] = 4300;

            distTable[4] = 500;
            rpmTable[4] = 4800;
        }
        // DISTANCE(CM) | VELOCITY (RPM)
        // 220		3400
        // 300		3700
        // 380		4000
        // 464		4300
        // 500		4800
        // LOW GOAL: 2000
    }

    /**
     * 
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.495;
    public static final double DRIVE_MAX_VELOCITY_METERS_PER_SECOND = .5;
    public static final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .5;
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final int CURRENT_LIMIT = 30;
    public static final double NORMAL_SPEED = .5;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5461;
    
    public final static class ControllerConstants {
        public static final int RIGHT_TRIGGER = 3;
        public static final int LEFT_TRIGGER = 2;
        public static final double TRIGGER_ACTIVATION_THRESHOLD = .3;
        public static final int POV_ANGLE_UP = 0;
        public static final int POV_ANGLE_LEFT = 270;
        public static final int POV_ANGLE_RIGHT = 90;
    }

    public final static class ConveyorConstants {
        public static final int LOWER_CONVEYOR = 15;
        public static final double CONVEYOR_SPEED = 0.5;
    }

    public final static class IntakeConstants {
        // intake things
        public static final int INTAKE_MOTOR = 10;
        public static final double INTAKE_SPEED = -.85;
        public static final double SPIT_OUT_BALL = .85;

        //pneumatics things
        public static final int FORWARD_CHANNEL_1 = 0;
        public static final int REVERSE_CHANNEL_1 = 7;
        public static final int FORWARD_CHANNEL_2 = 8;
        public static final int REVERSE_CHANNEL_2 = 15;
    }

    public final static class HangConstants {
        public static final int LEFT_HANG_MOTOR = 13; // left steer offset
        public static final int RIGHT_HANG_MOTOR = 14;
        public static final double HANG_SPEED = .6;
        public static final double LOWER_SPEED = -1;

        public static final double LIFT_HEIGHT = 53.29;
    }
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 18;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(127);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(35); //183

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 19;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(276);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR =  3;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(70);

    public final static class GameElementConstants {
        public final static double UPPER_HUB = 2.64; // meters
        public final static double LOWER_HUB = 1.04; // meters
        public final static double HIGH_HEIGHT = UPPER_HUB; // Subtract by robot height
        public final static double LOW_HEIGHT = LOWER_HUB;
    }

    public final static class AimbotConstants {
        public static final double baseSpeed = 0.1;
        public static final double minimumAdjustment = 0.5;
        public static final double Kp = 1; // OLD: 5
        public static final double Ki = 0;
        public static final double Kd = 0.001; // OLD: 0.8
    }

    public static final class AutoConstants {
        public static final double MAX_Speed_MetersPerSecond = 0.2;
        public static final double MAX_Acceleration_MetersPerSecondSquared = 0.2;
        public static final double MAX_AngularSpeed_RadiansPerSecond = Math.PI;
        public static final double Max_AngularSpeed_RadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 4.4558;
        public static final double kPYController = 4.4558;
        public static final double kPThetaController =  2.5;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
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
            public final static int BRIGHT = 1;
        }

        public final static double LIMELIGHT_HEIGHT = 34.5 * 2.54; // Converting from inches to cm
        public final static double LIMELIGHT_ANGLE = 20;
    }

    //Trajectory Config and path planner paths
    // public static final class TrajectoryPaths{
    //     public static final Trajectory BACK_UP_TO_COLLECT_BALL = PathPlanner.loadPath("BackUpToCollectBall",DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory TURN_1 = PathPlanner.loadPath("Turn_1", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory COLLECT_NEAR_HUMAN_PLAYER = PathPlanner.loadPath("CollectNearHumanPlayer", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory TURN_2 = PathPlanner.loadPath("Turn2", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory TURN_3 = PathPlanner.loadPath("Turn3", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory BACK_UP_TO_NEXT_BALL = PathPlanner.loadPath("BackUpToNextBall", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory TURN_4 = PathPlanner.loadPath("Turn4Blue", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory GET_FINAL_BALL = PathPlanner.loadPath("GetFinalBall", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory TURN_5 = PathPlanner.loadPath("Turn5", DRIVE_MAX_VELOCITY_METERS_PER_SECOND, DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    //     public static final Trajectory TEST_TRAJECTORY = PathPlanner.loadPath("Test2", 0.5, 0.5);
    // }

    public static final class TrajectoryPaths {
        // public static final Trajectory FourBallRed_BackUpToFirstBall = PathPlanner.loadPath("4BR_BackUpToFirstBall", 1, 1);
        // public static final Trajectory FourBallRed_GoToHumanPlayer = PathPlanner.loadPath("4BR_GoToHumanPlayer", 1, 1);
        // public static final Trajectory FourBallRed_Shoot = PathPlanner.loadPath("4BR_Shoot", 1, 1);
        // public static final Trajectory FourBallRed_Little = PathPlanner.loadPath("4BR_GoSlightlyFarther", 1, 1);
        // public static final Trajectory TestPath = PathPlanner.loadPath("Vaccum", 1, 1);
        public static final Trajectory BackToCollectBall = PathPlanner.loadPath ("BackToCollectBall", 1, 1);
        public static final Trajectory Turn1 = PathPlanner.loadPath("Turn1", 1, 1); 
        public static final Trajectory Step1 = PathPlanner.loadPath("Step1", 1, 1);
        public static final Trajectory Step2 = PathPlanner.loadPath("Step2", 1, 1);
       public static final Trajectory Step3 = PathPlanner.loadPath("Step3", 1, 1);
    }
}