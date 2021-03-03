// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * methods corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    public int stage=0;
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit()
    {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer(this);
    }

    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("EncoderL",robotContainer.m_robotDrive.getLeftEncoder().getDistance());
        SmartDashboard.putNumber("EncoderR",robotContainer.m_robotDrive.getRightEncoder().getDistance());
    }

    /** This method is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        robotContainer.m_robotCollector.stopAll();
        robotContainer.m_robotShooter.stopAll();
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        stage=0;
        SmartDashboard.putNumber("Stage",0);
        autonomousCommand = robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
//        SmartDashboard.putNumber("Stage",stage);
//        switch (stage){
//            case 1:
//                stage=2;
//                robotContainer.m_robotDrive.reverse();
//                robotContainer.getAutonomousCommand("BarrelSplit2.wpilib.json").schedule();
//                break;
//            case 3:
//                stage=4;
//                robotContainer.m_robotDrive.reverse();
//                robotContainer.getAutonomousCommand("BarrelSplit3.wpilib.json").schedule();
//                break;
//            case 5:
//                stage=6;
//                robotContainer.m_robotDrive.reverse();
//                robotContainer.getAutonomousCommand("BarrelSplit4.wpilib.json").schedule();
//                break;
//            default:
//        }
    }

    @Override
    public void teleopInit()
    {
        robotContainer.m_robotShooter.zeroAngle();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }

    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
