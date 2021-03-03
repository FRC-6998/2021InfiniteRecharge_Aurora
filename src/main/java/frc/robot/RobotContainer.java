// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;

import java.io.IOException;
import java.nio.file.Path;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    NetworkTableInstance nt=NetworkTableInstance.getDefault();
    XboxController controller = new XboxController(0);
    DriveSubsystem m_robotDrive = new DriveSubsystem();
    CollectSubsystem m_robotCollector = new CollectSubsystem();
    ShootSubsystem m_robotShooter = new ShootSubsystem(nt);
    Robot robot;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot robot) {
        this.robot = robot;
        configureButtonBindings();
        m_robotDrive.setDefaultCommand(new RunCommand(() -> {
            if (robot.isOperatorControlEnabled()) {
                teleopPeriodic();
            }
        }, m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID GenericHID}, or one of its
     * subclasses ({@link edu.wpi.first.wpilibj.Joystick Joystick} or
     * {@link edu.wpi.first.wpilibj.XboxController XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings() {

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(()->{
                    m_robotDrive.inverted=false;
                    m_robotCollector.rotationPanelCounterClockWise();
                    m_robotShooter.AutoShoot();
                    m_robotShooter.AutoShootStop();
                    m_robotCollector.AutoCollect();
                    m_robotDrive.reverse();
                }),
                getTrajectoryCommand("2020.wpilib.json"),
                new InstantCommand(()->{
                    m_robotCollector.stopAll();
                })
        );
//        return new SequentialCommandGroup(
//                getTrajectoryCommand("Barrel.wpilib.json")
//        );
    }

    private void teleopPeriodic() {
        m_robotDrive.arcadeDrive(-controller.getY(GenericHID.Hand.kLeft), controller.getX(GenericHID.Hand.kRight));
        m_robotShooter.setShootMotorsSpeed(controller.getTriggerAxis(GenericHID.Hand.kRight)-controller.getTriggerAxis(GenericHID.Hand.kLeft));
        if(controller.getRawButton(1)){
            m_robotShooter.enableTransfer();
        }else {
            m_robotShooter.disableTransfer();
        }
        if (controller.getRawButton(2)) {
            m_robotCollector.rotationPanelCounterClockWise();
        } else {
            m_robotCollector.stopPanel();
        }
        if(controller.getRawButton(3)){
            m_robotCollector.enableInTake();
        }else{
            m_robotCollector.disableInTake();
        }
    }

    private Command getTrajectoryCommand(String pathName) {
        Trajectory trajectory;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + pathName);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            RamseteCommand ramseteCommand = new RamseteCommand(
                    trajectory,
                    m_robotDrive::getPose,
                    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(DriveConstants.ksVolts,
                            DriveConstants.kvVoltSecondsPerMeter,
                            DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics,
                    m_robotDrive::getWheelSpeeds,
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    m_robotDrive::tankDriveVolts,
                    m_robotDrive
            );
            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(trajectory.getInitialPose());
            // Run path following command, then stop at the end.
            return ramseteCommand.andThen(() -> {
                m_robotDrive.tankDriveVolts(0, 0);
            });
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
            return new InstantCommand();
        }
    }
}
