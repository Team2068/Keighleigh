// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

        /**
         *
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static double MAX_VOLTAGE = 9;
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        // public static final double MAX_VELOCITY_METERS_PER_SECOND = (5880.0 / 60.0) *
        //                 SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        //                 SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 3; // this works lol
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.

        private final AHRS m_navx = new AHRS(Port.kUSB); // NavX connected over MXP

        SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
                        getGyroRotation(), new Pose2d(0, 0, new Rotation2d()));

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        private boolean isFieldOriented = false;

        public DrivetrainSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        
                m_frontLeftModule = Mk4SwerveModuleHelper.createNeo(
                                // This parameter is optional, but will allow you to see the current state of
                                // the module on the dashboard.
                                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(0, 0),
                                // This can either be STANDARD or FAST depending on your gear configuration
                                GearRatio.L2,
                                
                                FRONT_LEFT_MODULE_DRIVE_MOTOR, // This is the ID of the drive motor
                                
                                FRONT_LEFT_MODULE_STEER_MOTOR, // This is the ID of the steer motor
                                // This is the ID of the steer encoder
                                FRONT_LEFT_MODULE_STEER_ENCODER,
                                // This is how much the steer encoder is offset from true zero (In our case,
                                // zero is facing straight forward)
                                FRONT_LEFT_MODULE_STEER_OFFSET);

               
                m_frontRightModule = Mk4SwerveModuleHelper.createNeo(  // We will do the same for the other modules
                                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(2, 0),
                                GearRatio.L2,
                                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_MOTOR,
                                FRONT_RIGHT_MODULE_STEER_ENCODER,
                                FRONT_RIGHT_MODULE_STEER_OFFSET);

                m_backLeftModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(4, 0),
                                GearRatio.L2,
                                BACK_LEFT_MODULE_DRIVE_MOTOR,
                                BACK_LEFT_MODULE_STEER_MOTOR,
                                BACK_LEFT_MODULE_STEER_ENCODER,
                                BACK_LEFT_MODULE_STEER_OFFSET);

                m_backRightModule = Mk4SwerveModuleHelper.createNeo(
                                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                                .withSize(2, 4)
                                                .withPosition(6, 0),
                                GearRatio.L2,
                                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                BACK_RIGHT_MODULE_STEER_MOTOR,
                                BACK_RIGHT_MODULE_STEER_ENCODER,
                                BACK_RIGHT_MODULE_STEER_OFFSET);
        }

        public void adjustGyro(Photon photon){
                // photon.camera.setPipelineIndex(LimelightConstants.Pipelines.FIDICIAL);

                double gyroAngle = m_navx.getYaw();
                double distance = photon.getDistance(LimelightConstants.upperhub_height); // pass in fidicial Height
                double xOffset = photon.xOffset;

                double gyro_error_x = Math.abs((Math.cos(gyroAngle) * distance) - (Math.cos(xOffset) * distance));
                double gyro_error_y = Math.abs((Math.sin(gyroAngle) * distance) - (Math.sin(xOffset) * distance));
                
                double angleError = -(Math.atan(gyro_error_y / gyro_error_x));
                if (gyro_error_x > 1 || gyro_error_y > 1)
                        m_navx.setAngleAdjustment(angleError );
                        // m_navx.setAngleAdjustment(-(Math.atan(gyro_error_y / gyro_error_x)));

                SmartDashboard.putNumber("Gyro Error Angle", angleError);
                SmartDashboard.putNumber("Gyro Error X", gyro_error_x);
                SmartDashboard.putNumber("Gyro Error Y", gyro_error_y);

                
                // photon.camera.setPipelineIndex(LimelightConstants.Pipelines.FIDICIAL);
        }

        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        public Rotation2d getGyroRotation() {
                // Invert NavX angle to make counter-clockwise positive
                return Rotation2d.fromDegrees(Math.IEEEremainder(m_navx.getYaw(), 360));
        }

        public SwerveDriveKinematics getKinematics() {
                return m_kinematics;
        }

        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        public void adjustOdometry(Photon photon){
                double distance = photon.getDistance(139.7); // Pass in Fidicial height
                double xOffset = photon.xOffset;
                Pose2d currentPose = m_odometry.getPoseMeters();

                Translation2d camTrans = new Translation2d((Math.cos(xOffset) * distance), (Math.sin(xOffset) * distance));

                double odometry_err_x = currentPose.getX() - camTrans.getX();
                double odometry_err_y = currentPose.getY() - camTrans.getY();
        
                if (odometry_err_x > 1 || odometry_err_y > 1)
                m_odometry.resetPosition(new Pose2d(camTrans, currentPose.getRotation()), getGyroRotation());
        }

	public void resetOdometry() {
                zeroGyroscope();
		m_odometry.resetPosition(new Pose2d(), getGyroRotation());
	}


        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }


	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometryWithPose2d(Pose2d pose) {
		m_odometry.resetPosition(pose, pose.getRotation());
	}

        public void setModuleStates(SwerveModuleState[] states) {
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());
                m_odometry.update(getGyroRotation(), states);
        }

        public boolean getFieldOriented() {
                return isFieldOriented;
        }

        public void toggleFieldOriented() {
                isFieldOriented = !isFieldOriented;
        }

        public void turboSpeed() {
                MAX_VOLTAGE = 12;
        }

        public void slowSpeed() {
                MAX_VOLTAGE = 5;
        }

        public void standardSpeed() {
                MAX_VOLTAGE = 9;
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                setModuleStates(states);
                Pose2d pose = getPose();
                SmartDashboard.putNumber("X pos", pose.getX());
                SmartDashboard.putNumber("Y pos", pose.getY());
                SmartDashboard.putNumber("Odometry rotation", pose.getRotation().getDegrees());
                SmartDashboard.putString("Drive Mode", isFieldOriented ? "Field" : "Robot");
        }

}