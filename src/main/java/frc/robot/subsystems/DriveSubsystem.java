package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
    public boolean inverted = false;

    private double lastSpeed = 0;

    // The motors on the left side of the drive.
    private final SpeedControllerGroup m_leftMotors =
            new SpeedControllerGroup(new WPI_TalonSRX(DriveConstants.kMotor_L_Channel1),
                    new WPI_VictorSPX(DriveConstants.kMotor_L_Channel2));

    // The motors on the right side of the drive.
    private final SpeedControllerGroup m_rightMotors =
            new SpeedControllerGroup(new WPI_TalonSRX(DriveConstants.kMotor_R_Channel1),
                    new WPI_VictorSPX(DriveConstants.kMotor_R_Channel2));

    // The robot's drive
    private DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The left-side drive encoder
    private Encoder m_leftEncoder =
            new Encoder(DriveConstants.kEncoder_L_Channel1, DriveConstants.kEncoder_L_Channel2,
                    DriveConstants.kEncoder_L_Reversed);

    // The left-side drive encoder
    private Encoder m_rightEncoder =
            new Encoder(DriveConstants.kEncoder_R_Channel1, DriveConstants.kEncoder_R_Channel2,
                    DriveConstants.kEncoder_R_Reversed);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /**
     * Creates a new DriveSubsystem.
     */
    public DriveSubsystem() {
        resetEncoders();
        // Sets the distance per pulse for the encoders
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoder_DistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoder_DistancePerPulse);
        m_odometry = new DifferentialDriveOdometry(NavX.navx.getRotation2d());
    }

    public void setSafetyEnabled(boolean enabled){
        m_drive.setSafetyEnabled(enabled);
    }

    @Override
    public void periodic() {
        if(lastSpeed!=0){
            m_drive.arcadeDrive(lastSpeed,0);
        }
        // Update the odometry in the periodic block
        m_odometry.update(NavX.navx.getRotation2d(), m_leftEncoder.getDistance(),
                m_rightEncoder.getDistance());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, NavX.navx.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
        lastSpeed=fwd;
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(inverted?-rightVolts:leftVolts);
        m_rightMotors.setVoltage(inverted?leftVolts:-rightVolts);
        m_drive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        NavX.navx.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return NavX.navx.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -NavX.navx.getRate();
    }

    public void reverse(){
        inverted=!inverted;
        m_leftEncoder.close();
        m_rightEncoder.close();
        m_drive.close();
        if(inverted){
            m_drive = new DifferentialDrive(m_rightMotors,m_leftMotors);
            m_leftEncoder = new Encoder(DriveConstants.kEncoder_R_Channel1, DriveConstants.kEncoder_R_Channel2,
                    !DriveConstants.kEncoder_R_Reversed, CounterBase.EncodingType.k1X);
            m_rightEncoder = new Encoder(DriveConstants.kEncoder_L_Channel1, DriveConstants.kEncoder_L_Channel2,
                    !DriveConstants.kEncoder_L_Reversed, CounterBase.EncodingType.k1X);
        }else{
            m_drive = new DifferentialDrive(m_leftMotors,m_rightMotors);
            m_leftEncoder = new Encoder(DriveConstants.kEncoder_L_Channel1, DriveConstants.kEncoder_L_Channel2,
                    DriveConstants.kEncoder_L_Reversed, CounterBase.EncodingType.k1X);
            m_rightEncoder = new Encoder(DriveConstants.kEncoder_R_Channel1, DriveConstants.kEncoder_R_Channel2,
                    DriveConstants.kEncoder_R_Reversed, CounterBase.EncodingType.k1X);
        }
        resetEncoders();
        m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoder_DistancePerPulse);
        m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoder_DistancePerPulse);
    }
}