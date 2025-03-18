package frc.robot;

public class Utils {
    public static double sensitivity (double in, double a) {
        //ax^3+(1-a)x
        return ((a*in*in*in)+(1-a)*in);
    }
}
