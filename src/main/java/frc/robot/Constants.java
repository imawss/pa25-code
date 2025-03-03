package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

@SuppressWarnings("unused")
public class Constants {

    public static class SwerveDrive {
        public static double kWheelDiameter = 0.0508;
        public static double kWheelCircumference = Math.PI * kWheelDiameter;
        public static final double kGearRatio = 6.75;
        public static final double kSteerGearRatio = 21.1267;
        public static final double kDriveEncoderRot2Meter = kWheelCircumference / kGearRatio;
        public static final double kTurningEncoderRot2Rad = kSteerGearRatio / (Math.PI * 2);
        
        public static final boolean kDriveFLInverted = false;
        public static final boolean kDriveFRInverted = false;
        public static final boolean kDriveRRInverted = false;
        public static final boolean kDriveRLInverted = false;

        public static final boolean kSteerFLInverted = false;
        public static final boolean kSteerFRInverted = false;
        public static final boolean kSteerRRInverted = false;
        public static final boolean kSteerRLInverted = false;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 1.16; //%20 
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static int kEncoderReso = 4096;
        public static double kDriveMotorMaxSpeed = 6000;

        public static double kWheelBase = 0.57785;
        public static double kTrackWidth = 0.57785;

        public static double kFLOffset = -0.11181640625;
        public static double kFROffset = -0.46923828125;
        public static double kRLOffset = 0.2772216796875;
        public static double kRROffset = -0.177001953125;

        public static class DriveMotorPID {
            public static double kP = 0.2;
            public static double kI = 0.0;
            public static double kD = 0.1;
        }

        public static class SteerMotorPID {
            public static double kP = 0.5;
            public static double kI = 0.1;
            public static double kD = 0.0;
        }
    }

    public static class CANIds {
        public static int DRIVE_FL = 4;
        public static int DRIVE_FR = 0;
        public static int DRIVE_RL = 6;
        public static int DRIVE_RR = 2;

        public static int STEER_FL = 5;
        public static int STEER_FR = 1;
        public static int STEER_RL = 7;
        public static int STEER_RR = 3;

        public static int CANCODER_FL = 10;
        public static int CANCODER_FR = 8;
        public static int CANCODER_RL = 11;
        public static int CANCODER_RR = 9;

        public static int PIGEON = 12;
    }

    public static class DriverConstants{
        public static final int kDriverControllerPort = 0;
        public static final double kDeadband = 0.05;
    }
}
