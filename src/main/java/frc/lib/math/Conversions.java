package frc.lib.math;

public class Conversions {
    
    public static double RPSToMPS(double wheelRPS, double circumference){
        return wheelRPS * circumference;
    }

    public static double MPSToRPS(double wheelMPS, double circumference){
        return wheelMPS / circumference;
    }

    public static double rotationsToMeters(double wheelRotations, double circumference){
        return wheelRotations * circumference;
    }

    public static double metersToRotations(double wheelMeters, double circumference){
        return wheelMeters / circumference;
    }

    public static double motorRPSToMPS(double motorRPS, double circumference, double gearRatio){
        double wheelRPS = motorRPS / gearRatio;
        return wheelRPS * circumference;
    }

    public static double MPSToMotorRPS(double wheelMPS, double circumference, double gearRatio){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS * gearRatio;
    }

    public static double motorRotationsToMeters(double motorRotations, double circumference, double gearRatio){
        double wheelRotations = motorRotations / gearRatio;
        return wheelRotations * circumference;
    }

    public static double metersToMotorRotations(double wheelMeters, double circumference, double gearRatio){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations * gearRatio;
    }
}