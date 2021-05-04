package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ShootConstants;

public class ShootSubsystem extends SubsystemBase {
    private final CANSparkMax m_shoot_1 = new CANSparkMax(ShootConstants.kMotor_Shoot_Channel1, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax m_shoot_2 = new CANSparkMax(ShootConstants.kMotor_Shoot_Channel2, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SpeedControllerGroup m_shoots = new SpeedControllerGroup(m_shoot_1,m_shoot_2);
    private final WPI_VictorSPX m_Transfer = new WPI_VictorSPX(ShootConstants.kMotor_Shoot_Transfer);
    private final WPI_VictorSPX m_Shoot_Angle = new WPI_VictorSPX(ShootConstants.kMotor_Shoot_Angle_Channel);
    private final PIDController pid_Shoot_Angle = new PIDController(ShootConstants.kPID_Shoot_Angle_P,0,0);
    private final PIDController pid_Rotation = new PIDController(ShootConstants.kPID_Rotation_P,0, ShootConstants.kPID_Rotation_D);
    private final WPI_VictorSPX m_Shoot_Rotation = new WPI_VictorSPX(ShootConstants.kMotor_Shoot_Rotation_Channel);
    private final Servo s_Transfer = new Servo(0);
    private final Encoder angleEncoder = new Encoder(
            ShootConstants.kEncoder_Shoot_Angle_ChannelA,
            ShootConstants.kEncoder_Shoot_Angle_ChannelB,
            ShootConstants.kEncoder_Shoot_Angle_Reversed);

    private final DigitalInput upLimit = new DigitalInput(ShootConstants.kDigital_Limit_UP);
    private final DigitalInput downLimit = new DigitalInput(ShootConstants.kDigital_Limit_DOWN);
    private boolean zeroing = false;
    private boolean autoAlignment = true;
    private final NetworkTableInstance nt;
    public ShootSubsystem(NetworkTableInstance networkTableInstance) {
        pid_Shoot_Angle.setSetpoint(ShootConstants.kAngleOffset);
        pid_Rotation.setSetpoint(-ShootConstants.kRotationOffset);
        nt=networkTableInstance;
        m_Shoot_Angle.setInverted(true);
        m_shoot_1.setInverted(true);
        s_Transfer.setAngle(0);

//        m_shoot_1.setSmartCurrentLimit(30);
//        m_shoot_2.setSmartCurrentLimit(30);
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void periodic() {
        NetworkTable limelight = nt.getTable("limelight");
        boolean hasTarget=limelight.getEntry("tv").getDouble(0)>=1.0;
        if(autoAlignment){
            // PID控制水平轉向角度
            double currentRotationOutput=pid_Rotation.calculate(hasTarget?limelight.getEntry("tx").getDouble(-ShootConstants.kRotationOffset):-ShootConstants.kRotationOffset);
            m_Shoot_Rotation.set(-currentRotationOutput);
        }
        // PID閉環控制仰角
        double currentAngleOutput=zeroing?0.3:-pid_Shoot_Angle.calculate(hasTarget?limelight.getEntry("ty").getDouble(ShootConstants.kAngleOffset):ShootConstants.kAngleOffset);
        currentAngleOutput=MathUtil.clamp(currentAngleOutput,-0.3,0.3);
        if (!downLimit.get() && currentAngleOutput > 0) {
            if(zeroing){
                zeroing=false;
            }
            angleEncoder.reset();
            currentAngleOutput=0;
        }
        if (!upLimit.get() && currentAngleOutput<0) {
            currentAngleOutput=0;
        }
        m_Shoot_Angle.set(currentAngleOutput);
    }
    public void zeroAngle(){
        zeroing=true;
        m_Shoot_Angle.set(0.3);
    }
    public void stopAll(){
        m_Shoot_Angle.set(0);
        m_Shoot_Rotation.set(0);
        m_shoots.set(0);
    }
    public void enableShootMotors(){
        m_shoots.set(1);
    }
    public void disableShootMotors(){
        m_shoots.set(0);
    }
    public boolean isShootMotorsEnabled(){
        return m_shoots.get()!=0;
    }
    public void enableAutoAlignment(){
        autoAlignment=true;
        m_Shoot_Rotation.set(0);
    }
    public void disableAutoAlignment(){
        autoAlignment=false;
        m_Shoot_Rotation.set(0);
    }
    public void setRotationSpeed(double speed){
        if(autoAlignment) return;
        m_Shoot_Rotation.set(speed);
    }
    public void enableTransferMotor(){
        m_Transfer.set(0.8);
        s_Transfer.setAngle(70);
    }
    public void disableTransferMotor(){
        m_Transfer.set(0);
        s_Transfer.setAngle(0);
    }
    public boolean isTransferMotorEnabled(){
        return m_Transfer.get()!=0;
    }
}

