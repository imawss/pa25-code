package frc.robot;

public class Constants {
    
    public static class SwerveDrive{
        public static double WHEEL_DIAMETER = 0.0508;
        public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static double DRIVE_MOTOR_MAX_SPEED = 6000;

        public static class DriveMotorPID {
            public static double P = 0.2;
            public static double I = 0.0;
            public static double D = 0.1;            
        }

        public static class SteerMotorPID {
            public static double P = 0.5;
            public static double I = 0.1;
            public static double D = 0.0;            
        }
    }

    public static class Kraken {
        public static int ENCODER_RESOLUTION = 4096;        
    }

    public static class RobotConstants {
        public static double WHEEL_BASE, TRACK_WIDTH = 0.57785;
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
