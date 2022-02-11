package frc.robot;

import frc.robot.Constants; 

public final class JoyUtil {
    /* ++ we'll use this class to write methods that help us process joystick inputs
    * and will mostly be for drive train things, and will include things like:
    * deadzone functions and joystick input curves, and anything else we need. 
    */
    
    public static double posWithDeadzone(double pos) {
        // ++ takes input and compares it to deadzone size
        // returns joystick size if it's greater than the deadzone, 0 otherwise

        if (Math.abs(pos) >= Constants.Joysticks.deadZoneSize ) {
            return pos;
        } else {
            return 0.0; 
        }
    }

    public static double joyCurve(double pos) {
        // ++ this method will take the linear joystick input and puts it into a polynomial curve

        double a = Constants.Joysticks.aCoeff; 
        double b = Constants.Joysticks.bCoeff;
        int firstPower = Constants.Joysticks.firstPower; 
        int secondPower = Constants.Joysticks.secondPower; 

        return ( (a * (Math.pow(pos,firstPower))) + (b * (Math.pow(pos,secondPower))) ); 
        
        }




}
