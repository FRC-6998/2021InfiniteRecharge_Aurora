package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ShootSubsystem extends SubsystemBase {
    private final CANSparkMax m_shoot_1 = new CANSparkMax(ShootConstants.kMotor_Shoot_Channel1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_shoot_2 = new CANSparkMax(ShootConstants.kMotor_Shoot_Channel2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SpeedControllerGroup m_shoots = new SpeedControllerGroup(m_shoot_1,m_shoot_2);
    private final WPI_VictorSPX m_Shoot_Angle = new WPI_VictorSPX(ShootConstants.kMotor_Shoot_Angle_Channel);
    private final WPI_VictorSPX m_Shoot_Rotation = new WPI_VictorSPX(ShootConstants.kMotor_Shoot_Rotation_Channel);
    public ShootSubsystem() {
        m_shoot_2.setInverted(true);
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void enable(){

    }
    public void disable(){

    }
}

