// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.RunTrajectoryCommand;
import frc.robot.subsystems.CollectSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    Joystick controller1 = new Joystick(0);
    Joystick controller2 = new Joystick(1);
    DriveSubsystem m_robotDrive = new DriveSubsystem();
    CollectSubsystem m_robotCollector = new CollectSubsystem();
    ShootSubsystem m_robotShooter = new ShootSubsystem(nt);
    Robot robot;
    CANSparkMax hang = new CANSparkMax(22, CANSparkMaxLowLevel.MotorType.kBrushless);
    WPI_VictorSPX hangLine = new WPI_VictorSPX(10);
    WPI_VictorSPX hangBalance = new WPI_VictorSPX(11);

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
        // 搖桿1
        new JoystickButton(controller1, OIConstants.kReverseBtn).whenPressed(() -> m_robotDrive.reverse());
        new JoystickButton(controller1, 7)
                .whenPressed(() -> m_robotCollector.enableInTake(false))
                .whenReleased(() -> m_robotCollector.disableInTake());
        new JoystickButton(controller1, 8)
                .whenPressed(() -> m_robotCollector.rotationPanelCounterClockWise())
                .whenReleased(() -> m_robotCollector.stopPanel());
        new JoystickButton(controller1, 5)
                .whenPressed(() -> m_robotCollector.enableInTake(true))
                .whenReleased(() -> m_robotCollector.disableInTake());
        new JoystickButton(controller1, 6)
                .whenPressed(() -> m_robotCollector.setPanelMotorSpeed(-0.8))
                .whenReleased(() -> m_robotCollector.stopPanel());

        // 搖桿2
        new POVButton(controller2, 180)
                .whenPressed(() -> m_robotShooter.enableAutoAlignment(), m_robotShooter);
                //.whenReleased(() -> m_robotShooter.disableAutoAlignment(), m_robotShooter);
        new POVButton(controller2, 90)
                .whenPressed(() -> {
                    m_robotShooter.disableAutoAlignment();
                    m_robotShooter.setRotationSpeed(ShootConstants.kMotor_Manual_Rotation_Speed);
                }, m_robotShooter)
                .whenReleased(() -> {
                    m_robotShooter.setRotationSpeed(0);
                    m_robotShooter.enableAutoAlignment();
                },m_robotShooter);
        new POVButton(controller2, 270)
                .whenPressed(() -> {
                    m_robotShooter.disableAutoAlignment();
                    m_robotShooter.setRotationSpeed(-ShootConstants.kMotor_Manual_Rotation_Speed);
                }, m_robotShooter)
                .whenReleased(() -> {
                    m_robotShooter.setRotationSpeed(0);
                    m_robotShooter.enableAutoAlignment();
                },m_robotShooter);
        // 啟用射擊馬達
        new JoystickButton(controller2, OIConstants.kShootBtn)
                .whenPressed(new InstantCommand(() -> m_robotShooter.enableShootMotors()))
                .whenReleased(new InstantCommand(() -> m_robotShooter.disableShootMotors()));
        // 強制退球
        new JoystickButton(controller2, OIConstants.kShootReverseBtn)
                .whenPressed(new InstantCommand(() -> m_robotCollector.setPanelMotorSpeed(-0.8), m_robotCollector))
                .whenReleased(new InstantCommand(() -> m_robotCollector.stopPanel(), m_robotCollector));
        // 同時啟用轉盤與輸送馬達
        new JoystickButton(controller2, OIConstants.kRotationPanelAndEnableTransferBtn)
                .whenPressed(() -> {
                    if (!m_robotShooter.isShootMotorsEnabled()) return;
                    m_robotCollector.rotationPanelCounterClockWise();
                    m_robotShooter.enableTransferMotor();
                }, m_robotCollector, m_robotShooter)
                .whenReleased(() -> {
                    m_robotCollector.stopPanel();
                    m_robotShooter.disableTransferMotor();
                });
        new JoystickButton(controller2, 9)
                .whenPressed(() -> hang.set(-1))
                .whenReleased(() -> hang.set(0));
        new JoystickButton(controller2, 11)
                .whenPressed(() -> hang.set(1))
                .whenReleased(() -> hang.set(0));
        new JoystickButton(controller2, 10)
                .whenPressed(() -> hangLine.set(1))
                .whenReleased(() -> hangLine.set(0));
        new JoystickButton(controller2, 12)
                .whenPressed(() -> hangLine.set(-1))
                .whenReleased(() -> hangLine.set(0));
        new JoystickButton(controller2, 7)
                .whenPressed(() -> hangBalance.set(-1))
                .whenReleased(() -> hangBalance.set(0));
        new JoystickButton(controller2, 8)
                .whenPressed(() -> hangBalance.set(1))
                .whenReleased(() -> hangBalance.set(0));
        new JoystickButton(controller2, 6)
                .whenPressed(() -> {
                    m_robotCollector.enableInTake(false);
                    m_robotCollector.rotationPanelCounterClockWise();
                })
                .whenReleased(() -> {
                    m_robotCollector.stopPanel();
                    m_robotCollector.disableInTake();
                });
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        switch (robot.autoSelected) {
            case Robot.AUTO_2021_Full:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            m_robotDrive.inverted = false;
                            m_robotCollector.enableInTake(false);
                            m_robotCollector.rotationPanelCounterClockWise();
                        }),
                        new RunTrajectoryCommand(m_robotDrive, "AutoFullSplit1.wpilib.json"),
                        new InstantCommand(() -> {
                            m_robotCollector.disableInTake();
                            m_robotCollector.stopPanel();
                        }),
                        new InstantCommand(() -> m_robotDrive.reverse()),
                        new RunTrajectoryCommand(m_robotDrive, "AutoFullSplit2.wpilib.json")
                );
            case Robot.AUTO_2021_Only_Leave:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            m_robotDrive.inverted = false;
                            m_robotShooter.enableShootMotors();
                            //m_robotDrive.arcadeDrive(1,0);
                        }),
                        new DelayCommand(3),
                        new InstantCommand(()->{
                            m_robotShooter.enableTransferMotor();
                            m_robotCollector.rotationPanelCounterClockWise();
                        }),
                        new DelayCommand(9),
                        new InstantCommand(()->{
                            m_robotShooter.disableShootMotors();
                            m_robotCollector.stopPanel();
                            m_robotShooter.disableTransferMotor();
                            m_robotDrive.arcadeDrive(1,0);
                        }),
                        new DelayCommand(1),
                        new InstantCommand(()->{
                            m_robotDrive.arcadeDrive(0,0);
                        })
                );
            case Robot.AUTO_Barrel:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> m_robotDrive.inverted = false),
                        new RunTrajectoryCommand(m_robotDrive, "BarrelRacing.wpilib.json")
                );
            case Robot.AUTO_Bounce:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> m_robotDrive.inverted = false),
                        new RunTrajectoryCommand(m_robotDrive, "BounceSplit1.wpilib.json"),
                        new InstantCommand(() -> m_robotDrive.reverse()),
                        new RunTrajectoryCommand(m_robotDrive, "BounceSplit2.wpilib.json"),
                        new InstantCommand(() -> m_robotDrive.reverse()),
                        new RunTrajectoryCommand(m_robotDrive, "BounceSplit3.wpilib.json"),
                        new InstantCommand(() -> m_robotDrive.reverse()),
                        new RunTrajectoryCommand(m_robotDrive, "BounceSplit4.wpilib.json")
                );
            case Robot.AUTO_Slalom:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> m_robotDrive.inverted = false),
                        new RunTrajectoryCommand(m_robotDrive, "sladom.wpilib.json")
                );
            case Robot.AUTO_GalasticA:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            m_robotDrive.inverted = false;
                            m_robotCollector.enableInTake(false);
                            m_robotCollector.rotationPanelCounterClockWise();
                        }),
                        new RunTrajectoryCommand(m_robotDrive, "GalasticA.wpilib.json"),
                        new InstantCommand(() -> {
                            m_robotCollector.disableInTake();
                            m_robotCollector.stopPanel();
                        })
                );
            case Robot.AUTO_GalasticB:
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            m_robotDrive.inverted = false;
                            m_robotCollector.enableInTake(false);
                            m_robotCollector.rotationPanelCounterClockWise();
                        }),
                        new RunTrajectoryCommand(m_robotDrive, "GalasticB.wpilib.json"),
                        new InstantCommand(() -> {
                            m_robotCollector.disableInTake();
                            m_robotCollector.stopPanel();
                        })
                );
            default:
                return new InstantCommand();
        }
    }

    private void teleopPeriodic() {
        // 搖桿1
        double ySpeed = -controller1.getY(GenericHID.Hand.kLeft);
        ySpeed = Math.abs(ySpeed) >= 0.2 ? ySpeed : 0;
        double zRotation = controller1.getZ();
        zRotation = Math.abs(zRotation) >= 0.2 ? 0.8* zRotation : 0;
        m_robotDrive.arcadeDrive(ySpeed, zRotation);
    }
}

