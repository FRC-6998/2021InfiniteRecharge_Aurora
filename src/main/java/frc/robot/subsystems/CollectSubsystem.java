package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class CollectSubsystem extends SubsystemBase {
    private final SpeedControllerGroup m_InTakes = new SpeedControllerGroup(
            new WPI_VictorSPX(CollectorConstants.kMotor_Channel1),
            new WPI_VictorSPX(CollectorConstants.kMotor_Channel2)
    );

    private final WPI_VictorSPX m_Panel = new WPI_VictorSPX(CollectorConstants.kMotor_Panel);

    public CollectSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
    public void enableInTake(){
        m_InTakes.set(CollectorConstants.kMotor_InTakes_Speed);
    }
    public void disableInTake(){
        m_InTakes.set(0);
    }
    public void rotationPanelClockWise(){
        m_Panel.set(CollectorConstants.kMotor_Panel_Speed);
    }
    public void rotationPanelCounterClockWise(){
        m_Panel.set(-CollectorConstants.kMotor_Panel_Speed);
    }
    public void reversePanel(){
        m_Panel.set(-m_Panel.get());
    }
}

