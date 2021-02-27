package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class Constants {
    public static class DriveConstants{
        public static final int kMotor_L_Channel1 = 1;
        public static final int kMotor_L_Channel2 = 2;
        public static final int kMotor_R_Channel1 = 3;
        public static final int kMotor_R_Channel2 = 4;

        public static final int kEncoder_L_Channel1 = 4;
        public static final int kEncoder_L_Channel2 = 5;
        public static final boolean kEncoder_L_Reversed = true;
        public static final int kEncoder_R_Channel1 = 0;
        public static final int kEncoder_R_Channel2 = 1;
        public static final boolean kEncoder_R_Reversed = false;
        public static final double kEncoder_DistancePerPulse = 0.478/360;

        // values for your robot.
        public static final double ksVolts = 0.799;
        public static final double kvVoltSecondsPerMeter = 3.01;
        public static final double kaVoltSecondsSquaredPerMeter = 0.348;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.01;

        public static final double kTrackWidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackWidthMeters);
    }
    public static class ShootConstants{
        //TODO 設定
        public static final int kMotor_Shoot_Channel1=0;
        public static final int kMotor_Shoot_Channel2=0;
        public static final int kMotor_Shoot_Angle_Channel=0;
        public static final int kMotor_Shoot_Rotation_Channel=0;

        public static final int kEncoder_Shoot_Angle_ChannelA=0;
        public static final int kEncoder_Shoot_Angle_ChannelB=0;
        public static final boolean kEncoder_Shoot_Angle_Reversed=false;

        public static final int kMotor_Shoots_Speed=1;
    }
    public static class OIConstants{

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
        public static final int kMotor_Channel1=0;
        public static final int kMotor_Channel2=0;
        public static final double kMotor_InTakes_Speed = 0 ;

        public static final int kMotor_Panel = 0;
        public static final double kMotor_Panel_Speed = 0.3;
    }
}
