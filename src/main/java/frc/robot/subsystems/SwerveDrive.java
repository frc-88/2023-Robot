// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DRIVETRAIN_TRACKWIDTH_METERS;
import static frc.robot.Constants.DRIVETRAIN_WHEELBASE_METERS;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drive.GrantDriveCommand;
import frc.robot.commands.drive.SwerveDriveCommand;
import frc.robot.util.controllers.DriverController;
import frc.robot.util.drive.DriveUtils;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class SwerveDrive extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * 
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;

        public static final int NUM_MODULES = 4;

        /**
         * The maximum velocity of the robot in meters per second.
         * 
         * This is a measure of how fast the robot should be able to drive in a straight
         * line. The calculated theoritical maximum is used below.
         */

        /*public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                        SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
                        SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;*/

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.9;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * 
         * This is a measure of how fast the robot can rotate in place.
         * The calculated theoritical maximum is used below.
         */
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        private DoublePreferenceConstant p_frontLeftOffset = new DoublePreferenceConstant("Drive Front Left Offset",
                        0.0);
        private DoublePreferenceConstant p_frontRightOffset = new DoublePreferenceConstant("Drive Front Right Offset",
                        0.0);
        private DoublePreferenceConstant p_backLeftOffset = new DoublePreferenceConstant("Drive Back Left Offset", 0.0);
        private DoublePreferenceConstant p_backRightOffset = new DoublePreferenceConstant("Drive Back Right Offset",
                        0.0);

        private final SlewRateLimiter filterX = new SlewRateLimiter(3.0);
        private final SlewRateLimiter filterY = new SlewRateLimiter(3.0);

        private final AHRS m_navx;
        private SwerveModule m_frontLeftModule;
        private SwerveModule m_frontRightModule;
        private SwerveModule m_backLeftModule;
        private SwerveModule m_backRightModule;
        private SwerveModule[] m_modules;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        private double m_fieldOffset = 0.0;
        private SwerveDriveOdometry m_odometry;
        private Pose2d m_pose;

        private Pose2d m_traj_pose;
        private Pose2d m_traj_reset_pose;
        private Pose2d m_traj_offset;

        public SwerveDrive() {
                m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

                zeroGyroscope();
                configureModules();

                m_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
                m_traj_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
                m_odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(),
                                getSwerveModulePositions(),
                                m_pose);
       }

        public SwerveModulePosition[] getSwerveModulePositions() {
                return new SwerveModulePosition[] {
                                m_frontLeftModule.getPosition(),
                                m_frontRightModule.getPosition(),
                                m_backLeftModule.getPosition(),
                                m_backRightModule.getPosition(),
                };
        }

        public void configureModules() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                                -Math.toRadians(p_frontLeftOffset.getValue()));
                m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                                tab.getLayout("Front Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                                -Math.toRadians(p_frontRightOffset.getValue()));
                m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Left", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                                -Math.toRadians(p_backLeftOffset.getValue()));
                m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                                tab.getLayout("Back Right", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                                Mk4iSwerveModuleHelper.GearRatio.L2,
                                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                                -Math.toRadians(p_backRightOffset.getValue()));

                m_modules = new SwerveModule[] {m_frontLeftModule, m_frontRightModule, m_backLeftModule, m_backRightModule};

                m_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
                m_traj_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
                m_odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(),
                                getSwerveModulePositions(),
                                m_pose);
        }

        public SwerveModule[] getModules() {
                return m_modules;
        }

        public int getNumModules() {
                return m_modules.length;
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {
                m_navx.zeroYaw();
        }

        public AHRS getNavX() {
                return m_navx;
        }

        public Rotation2d getGyroscopeRotation() {
                // if (m_navx.isMagnetometerCalibrated()) {
                // // We will only get valid fused headings if the magnetometer is calibrated
                // return Rotation2d.fromDegrees(m_navx.getFusedHeading());
                // }

                // We have to invert the angle of the NavX so that rotating the robot
                // counter-clockwise makes the angle increase.
                return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
        }

        public void zeroOdometry() {
                resetOdometry(new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d()),
                                new Rotation2d());
        }

        public void resetOdometry(Pose2d startPose, Rotation2d startGyro) {
                m_odometry.resetPosition(startGyro,
                                getSwerveModulePositions(),
                                startPose);
                m_fieldOffset = startPose.getRotation().getDegrees();
        }

        public void resetTrajectoryPose(Pose2d startPose) {
                m_traj_reset_pose = m_odometry.getPoseMeters();
                m_traj_offset = startPose;
                m_fieldOffset = startPose.getRotation().getDegrees();
        }

        public void updateOdometry() {
                m_pose = m_odometry.update(getGyroscopeRotation(),
                                getSwerveModulePositions());
                if (m_traj_reset_pose == null || m_traj_offset == null) {
                        m_traj_pose = m_pose;
                } else {
                        Pose2d reset_relative_pose = m_pose.relativeTo(m_traj_reset_pose);
                        m_traj_pose = DriveUtils.relativeToReverse(reset_relative_pose, m_traj_offset);
                }
        }

        public Pose2d getTrajectoryOdometryPose() {
                return m_traj_pose;
        }

        public Pose2d getOdometryPose() {
                return m_pose;
        }

        public ChassisSpeeds getChassisSpeeds() {
                var frontLeftState = new SwerveModuleState(m_frontLeftModule.getDriveVelocity(),
                                new Rotation2d(m_frontLeftModule.getSteerAngle()));
                var frontRightState = new SwerveModuleState(m_frontRightModule.getDriveVelocity(),
                                new Rotation2d(m_frontRightModule.getSteerAngle()));
                var backLeftState = new SwerveModuleState(m_backLeftModule.getDriveVelocity(),
                                new Rotation2d(m_backLeftModule.getSteerAngle()));
                var backRightState = new SwerveModuleState(m_backRightModule.getDriveVelocity(),
                                new Rotation2d(m_backRightModule.getSteerAngle()));

                return kinematics.toChassisSpeeds(frontLeftState, frontRightState, backLeftState, backRightState);
        }

        public double getAccelerationEstimate() {
                //      
                return 0.0;
        }

        public double getStraightSpeed() {
                return Math.sqrt(Math.pow(m_chassisSpeeds.vxMetersPerSecond, 2)
                                + Math.pow(m_chassisSpeeds.vyMetersPerSecond, 2));
        }

        public void resetPosition(Pose2d pose) {
                resetOdometry(pose, getGyroscopeRotation());
        }

        public double getFieldOffset() {
                return m_fieldOffset;
        }

        public void stop() {
                drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        }

        public void drive(double vx, double vy, double angularVelocity) {
                drive(new ChassisSpeeds(vx, vy, angularVelocity));
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
                SwerveModuleState[] states = kinematics.toSwerveModuleStates(m_chassisSpeeds);
                setModuleStates(states);
        }

        public void setModuleStates(SwerveModuleState[] states) {
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());
        }

        public SwerveDriveCommand fieldOrientedDriveCommandFactory(SwerveDrive drive,
                        DriverController driverController) {
                SwerveDriveCommand swerveDrive;

                swerveDrive = new SwerveDriveCommand(drive,
                                () -> modifyAxis(filterY.calculate(driverController.getTranslationY()))
                                                * MAX_VELOCITY_METERS_PER_SECOND,
                                () -> modifyAxis(filterX.calculate(driverController.getTranslationX()))
                                                * MAX_VELOCITY_METERS_PER_SECOND,
                                () -> modifyAxis(driverController.getRotation())
                                                * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
                return swerveDrive;
        }

        public GrantDriveCommand grantDriveCommandFactory(SwerveDrive drive,
                        DriverController driverController) {
                GrantDriveCommand grantDrive;

                grantDrive = new GrantDriveCommand(drive,
                        () -> modifyAxis(filterY.calculate(((FrskyController) m_driverController).getLeftStickY())) * SwerveDrive.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> modifyAxis(((FrskyController) m_driverController).getRightStickX()),
                        () -> modifyAxis(((FrskyController) m_driverController).getRightStickY()),
                        () -> -modifyAxis(((FrskyController) m_driverController).getLeftStickX()) * SwerveDrive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                return grantDrive;
        }

        private double deadband(double value, double deadband) {
                if (Math.abs(value) > deadband) {
                        if (value > 0.0) {
                                return (value - deadband) / (1.0 - deadband);
                        } else {
                                return (value + deadband) / (1.0 - deadband);
                        }
                } else {
                        return 0.0;
                }

        }

        private double modifyAxis(double value) {
                // Deadband
                value = deadband(value, 0.05);

                // Square the axis
                value = Math.copySign(value * value, value);

                return value;
        }

        @Override
        public void periodic() {
                updateOdometry();

                SmartDashboard.putNumber("odomX", Units.metersToFeet(m_pose.getX()));
                SmartDashboard.putNumber("odomY", Units.metersToFeet(m_pose.getY()));
                SmartDashboard.putNumber("odomTheta", m_pose.getRotation().getDegrees());
                SmartDashboard.putNumber("field offset", m_fieldOffset);
                //SmartDashboard.putNumber("FLSteerCurrent", m_);
                //SmartDashboard.putNumber("FRSteerCurrent", m_);               
                //SmartDashboard.putNumber("BLSteerCurrent", m_);               
                //SmartDashboard.putNumber("BRSteerCurrent", m_);
                //SmartDashboard.putNumber("FLDriveCurrent", m_);
                //SmartDashboard.putNumber("FRDriveCurrent", m_);
                //SmartDashboard.putNumber("BLDriveCurrent", m_);
                //SmartDashboard.putNumber("BRDriveCurrent", m_);
        }
}