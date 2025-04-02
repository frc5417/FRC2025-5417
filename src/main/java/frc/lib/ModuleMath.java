package frc.lib;

public class ModuleMath {

    public static double normalizeDegrees(double degrees) {
        if (degrees < 0) {
            // if the angle is negative
            return -degrees % 360;
        }
        
        return degrees % 360;
    }
    
}
