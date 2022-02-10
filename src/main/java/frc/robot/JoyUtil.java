package frc.robot;

public final class JoyUtil {
    /* ++ we'll use this class to write methods that help us process joystick inputs
    * and will mostly be for drive train things, and will include things like:
    * deadzone functions and joystick input curves, and anything else we need. 
    */
    
    public static double deadzone(double axis) {
        //  ++ takes input and compares it to deadzone size
        // returns joystick size if it's greater than the deadzone, 0 otherwise

        if (axis > .075) {
            return axis;
        }
        else if (axis < -.075) {
            return axis;
        }
        else {
            return 0;
        }

        // if (Math.abs(axis) >= Constants.deadZoneSize) {
        //     return axis;
        // } else {
        //     return 0.0; 
        // }
    }

}
