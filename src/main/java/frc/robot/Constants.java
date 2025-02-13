package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {

    public static class SwerveDrive {
        public static double kWheelDiameter = 0.0508;
        public static double kWheelCircumference = Math.PI * kWheelDiameter;
        public static final double kGearRatio = 1 / 6.75;
        public static final double kDriveEncoderRot2Meter = kGearRatio * kWheelDiameter * Math.PI;
        public static final double kTurningEncoderRot2Rad = kGearRatio * Math.PI * 2;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        
        public static final boolean kDriveFLInverted = false;
        public static final boolean kDriveFRInverted = false;
        public static final boolean kDriveRRInverted = false;
        public static final boolean kDriveRLInverted = false;

        public static final boolean kSteerFLInverted = false;
        public static final boolean kSteerFRInverted = false;
        public static final boolean kSteerRRInverted = false;
        public static final boolean kSteerRLInverted = false;

        public static int kEncoderReso = 4096;
        public static double kDriveMotorMaxSpeed = 6000;

        public static double kWheelBase, kTrackWidth = 0.57785;

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
        public static int DRIVE_FL = 1;
        public static int DRIVE_FR = 2;
        public static int DRIVE_RL = 3;
        public static int DRIVE_RR = 4;

        public static int STEER_FL = 5;
        public static int STEER_FR = 6;
        public static int STEER_RL = 7;
        public static int STEER_RR = 8;

        public static int CANCODER_FL = 9;
        public static int CANCODER_FR = 10;
        public static int CANCODER_RL = 11;
        public static int CANCODER_RR = 12;

        public static int PIGEON = 20;
    }
}
