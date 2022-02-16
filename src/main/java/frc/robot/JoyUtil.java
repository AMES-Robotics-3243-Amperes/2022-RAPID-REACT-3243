package frc.robot;

import frc.robot.Constants; 

public final class JoyUtil {

    /* ++ we'll use this class to write methods that help us process joystick inputs
    * and will mostly be for drive train things, and will include things like:
    * deadzone functions and joystick input curves, and anything else we need. 
    */

    // ++ WE HAVE A BUNCH OF FUNCTIONS HERE, AND WE NEED TO APPLY THEM IN THE RIGHT ORDER
    // ++ check the "composeDriveJoyFunctions" method at the bottom to see the order this should be done in 
    // ++ (((I'm not putting it here to avoid multiple versions of the "correct" order)))

    
    public static double posWithDeadzone(double pos) {
        // ++ takes input and compares it to deadzone size
        // returns joystick size if it's greater than the deadzone, 0 otherwise

        if (Math.abs(pos) >= Constants.Joysticks.deadZoneSize ) {
            return pos;
        } else {
            return 0.0; 
        }
    }


    public static double driveLowPassFilter(double pos, double prevFilterJoy) {
        // ++ this method smoothes out the joystick input so 
        // ++ "prevFilterJoy" is the previous output of this function
        double filteredSpeed = ((Constants.Joysticks.driveLowPassFilterStrength * prevFilterJoy) + ((1- Constants.Joysticks.driveLowPassFilterStrength) * pos));
        return filteredSpeed;
    }


    public static double joyCurve(double pos) {
        // ++ this method will take the linear joystick input and puts it into a polynomial curve

        double a = Constants.Joysticks.aCoeff; 
        double b = Constants.Joysticks.bCoeff;
        int firstPower = Constants.Joysticks.firstPower; 
        int secondPower = Constants.Joysticks.secondPower; 

        return ( (a * (Math.pow(pos,firstPower))) + (b * (Math.pow(pos,secondPower))) ); 
    }


    public static double composeDriveJoyFunctions(double rawJoyPos, double prevFilterJoy){
        /* ++ this method will compose all the previous joy functions, so
        * THIS WILL BE THE ONLY METHOD USED for adjusting the drive joysticks
        *
        * ++ we need to apply the methods in the RIGHT ORDER
        * we want to do:
        * - deadzone
        * - low pass filtering
        * - joy curve
        * - dampen the output w/ a multiplier
        * and the order might have to be changed as we add more functions, 
        * but deadzone should probably stay first, and dampening should probably stay last
        */ 

        double withDead = posWithDeadzone(rawJoyPos);
        double withFilter = driveLowPassFilter(withDead, prevFilterJoy);
        double withCurve = joyCurve(withFilter); 
        double withDampened = withCurve * Constants.Joysticks.driveSpeedDamper; 

        // ++ I decided to make seperate variables for everything to make it a little more readable

        return withDampened;
        // ++ we return "withCurve" because the curve is the last method so far, and
        // it'll need to be changed if/when more functions are added
    }



}
