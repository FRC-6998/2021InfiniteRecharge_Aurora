package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static class DriveConstants{
        public static final int kMotor_L_Channel1 = 1;
        public static final int kMotor_L_Channel2 = 2;
        public static final int kMotor_R_Channel1 = 3;
        public static final int kMotor_R_Channel2 = 4;

        public static final int kEncoder_L_Channel1 = 2;
        public static final int kEncoder_L_Channel2 = 3;
        public static final boolean kEncoder_L_Reversed = true;
        public static final int kEncoder_R_Channel1 = 0;
        public static final int kEncoder_R_Channel2 = 1;
        public static final boolean kEncoder_R_Reversed = false;
        public static final double kEncoder_DistancePerPulse = 4 * Math.PI * 0.0254 / 360;

        // values for your robot.
        public static final double ksVolts = 0.642;
        public static final double kvVoltSecondsPerMeter = 3.66;
        public static final double kaVoltSecondsSquaredPerMeter = 0.203;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.66;

        public static final double kTrackWidthMeters = 0.609;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackWidthMeters);
    }
    public static class ShootConstants{
        //TODO 設定
        public static final int kMotor_Shoot_Channel1=1;
        public static final int kMotor_Shoot_Channel2=11;
        public static final int kMotor_Shoot_Angle_Channel=5;
        public static final int kMotor_Shoot_Rotation_Channel=9;
        public static final int kMotor_Shoot_Transfer=8;

        public static final int kEncoder_Shoot_Angle_ChannelA=4;
        public static final int kEncoder_Shoot_Angle_ChannelB=5;
        public static final boolean kEncoder_Shoot_Angle_Reversed=false;

        public static final int kDigital_Limit_UP = 7;
        public static final int kDigital_Limit_DOWN = 6;

        public static final double kPID_Shoot_Angle_P = 0.1;
        public static final double kPID_Rotation_P = 0.12;
        public static final double kPID_Rotation_D = 0;

        public static final double kMotor_Manual_Rotation_Speed = 0.8;

        // 向左調整為負 向右調整為正
        public static final double kRotationOffset = 1.75;
        // 向上調整為負 向下為正
        public static final double kAngleOffset = -2;

    }
    public static class OIConstants{
        // Controller 1
        public static final int kReverseBtn = 1;
        // Controller 2
        public static final int kShootBtn = 1;
        public static final int kShootReverseBtn = 5;
        public static final int kRotationPanelAndEnableTransferBtn = 2;
    }
    public static class NavX{
        public static final AHRS navx = new AHRS();
    }

    public static class AutoConstants{
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    public static class CollectorConstants {
        //TODO 設定
        public static final int kMotor_Channel=6;
        public static final double kMotor_InTakes_Speed = -0.9;

        public static final int kMotor_Panel = 7;
        public static final double kMotor_Panel_Speed = 0.3;
    }
}
