// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveTrain {
        // ++ MOTOR CONTORLLER IDS ---------------------------------------------------------
        // CAN
        public static final int frontLeftID = 4;
        public static final int frontRightID = 3;
        public static final int backLeftID = 2;
        public static final int backRightID = 1;
        // ++ ---------------------------------------------------------------
        // ~~ There are the positions of the mecanum wheels in meters
        public static final Translation2d frontLeftMeters = new Translation2d(0.257175,0.254);
        public static final Translation2d frontRightMeters = new Translation2d(0.257175,-0.254);
        public static final Translation2d backLeftMeters = new Translation2d(-0.257175,0.254);
        public static final Translation2d backRightMeters = new Translation2d(-0.257175,-0.254);
        // public static final Translation2d frontLeftMeters = new Translation2d(-0.254,0.257175);
        // public static final Translation2d frontRightMeters = new Translation2d(0.254,0.257175);
        // public static final Translation2d backLeftMeters = new Translation2d(-0.254,-0.257175);
        // public static final Translation2d backRightMeters = new Translation2d(0.254,-0.257175);

        // ~~ Radius of the wheels in meters
        public static final double wheelDiameter = 0.1524;
        // ~~ PID values for teleop mecanum drive
        public static final double teleopPGain = 0.2;
        public static final double teleopIGain = 0;
        public static final double teleopDGain = 0;
        // ~~ Conversion ratios for drivetrain encoders
            // ++ probably between the velocity of the wheel and the motor (idk which direction)
        public static final double velocityConversionRatio = ((2.4 * wheelDiameter * Math.PI)/(4 * 10.71 * 60));
        public static final double positionConversionRation = ((2.4 * wheelDiameter * Math.PI)/(4 * 10.71));
        // ~~ Speed error threshold for crash detection
        public static final double speedErrorThreshold = 0.1;
    }

    
    public static final class Joysticks {
        // ++ CONTROLLER IDS --------------------------------------------------------------------
        public static final int primaryControllerID = 0;
        public static final int secondaryControllerID = 1; 

        // ++ JOYSTICK AXIS IDS ---------------------------------------------------------------------
        // ++ these are maybe wrong/redundant with the xbox controllers/libraries?
        public static final int LeftJoystickX = 2;
        public static final int LeftJoystickY = 1;
        public static final int RightJoystickX = 3;
        public static final int RightJoystickY = 4;

        // ++ JOYSTICK BUTTON IDS -------------------------------------------------------------------
        public static final int A = 2;
        public static final int B = 3;
        public static final int X = 1;
        public static final int Y = 4;
        public static final int Start = 10;
        public static final int Back = 9;
        public static final int LeftBumper = 5;
        public static final int RightBumper = 6;
        public static final int LeftTrigger = 7;
        public static final int RightTrigger = 8;
        public static final int DpadXaxis = 5;
        public static final int DpadYaxis = 6;


        // ++ OTHER JOYSTICK CONSTANTS --------------------------------------------------------------
        public static final double deadZoneSize = 0.15;
        // ++ lowPassFilterStrength should be between 0 & 1. The closer it is to 1, the smoother it is. 
        public static final double driveLowPassFilterStrength = 0.88;
        public static final double rotationLowPassFilterStrength = 0.2;
        // ++ we probably don't want the speed dampers as finals incase we want a fastmode/to change them later
        public static final double driveSpeedDamper = 0.5; 
        public static final double rotationDamper = 0.5; 


        // ++ JOYSTICK CURVE CONSTANTS --------------------------------------------------------------
        public static final double aCoeff = 0.7;
        public static final int firstPower = 3;

        public static final int secondPower = 1; 
        public static final double bCoeff = (1.0 - aCoeff); 

    }

}
