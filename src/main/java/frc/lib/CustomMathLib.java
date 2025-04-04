package frc.lib;

public class CustomMathLib {

    public static double normalizeDegrees(double degrees) {
        while (degrees > 360) {
            degrees -= 360;
        }
        while (degrees < 0) {
            degrees += 360;
        }
        return degrees;
    }
}
