// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public final static double ROBOT_HEIGHT = 0.0;// adjust

    public final static class ShooterConstants {
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
    // public final static double UPPER_HUB = 2.64; //meters
    // public final static double LOWER_HUB = 1.04; //meters
    // public final static double HIGH_HEIGHT = UPPER_HUB - ROBOT_HEIGHT;
    // public final static double LOW_HEIGHT = LOWER_HUB - ROBOT_HEIGHT;
    // }
    public final static class ControllerConstants {
        public static final int RIGHT_TRIGGER = 3;
        public static final int LEFT_TRIGGER = 2;
        public static final double TRIGGER_ACTIVATION_THRESHOLD = .3;
        public static final int POV_ANGLE_UP = 0;
        public static final int POV_ANGLE_LEFT = 270;
        public static final int POV_ANGLE_RIGHT = 90;
    }
    public final static class ConveyorConstants {
        public static final int LOWER_CONVEYOR = 16;
        public static final int UPPER_CONVEYOR = 17;
        public static final double CONVEYOR_SPEED = 0.2;
    }
    public final static class IntakeConstants {
        //intake things
        public static final int INTAKE_MOTOR = 18;
        public static final double INTAKE_SPEED = .2;
        public static final double SPIT_OUT_BALL = -.2;
        //pneumatics things
        public static final int FORWARD_CHANNEL = 1;
        public static final int REVERSE_CHANNEL = 0;
    }

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 13; // FIXME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(75.0); // FIXME Measure and set front
                                                                                       // left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; // FIXME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; // FIXME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(153); // FIXME Measure and set front
                                                                                       // right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9; // FIXME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; // FIXME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10; // FIXME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(295); // FIXME Measure and set back left
                                                                                     // steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // FIXME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2; // FIXME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; // FIXME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(5); // FIXME Measure and set back right
                                                                                    // steer offset

    public final static class GameElementConstants {
        public final static double UPPER_HUB = 2.64; // meters
        public final static double LOWER_HUB = 1.04; // meters
        public final static double HIGH_HEIGHT = UPPER_HUB; // Subtract by robot height
        public final static double LOW_HEIGHT = LOWER_HUB;
    }

    public final static class AimbotConstants {
        public static final double baseSpeed = 0.5;
        public static final double minimumAdjustment = 1.1;
        public static final double Kp = 5;
        public static final double Ki = 0;
        public static final double Kd = 0.8;
    }

}
