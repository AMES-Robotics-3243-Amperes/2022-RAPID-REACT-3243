// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        // ++ these are the channels for the drivetran motors
        // CAN
        public static final int frontLeftID = 4;
        public static final int frontRightID = 3;
        public static final int backLeftID = 2;
        public static final int backRightID = 1;
    }

    public static final class IntakeIndexer {
        // ~~ these are the values for the intake/indexer motors
        // ~~ CAN IDs
        public static final int flyWheelID = 9;
        public static final int intakeMotorID = 8;
        public static final int indexMotorID = 7;
        // ~~ Dropped position
        public static final double intakeDropPos = 0.1;
        // ~~ Encoder conversion ratios to account for gearbox ratios
        public static final double flywheelVelocityConversionRatio = 1;
        public static final double intakeVelocityConversionRatio = (1/12);
        public static final double indexVelocityConversionRatio = (1/120);
        public static final double flywheelPositionConversionRatio = 1;
        public static final double intakePositionConversionRatio = (1/12);
        public static final double indexPositionConversionRatio = (1/120);
        // ~~ Accept and Rebuff Constants
        public static final double acceptRotations = 30;
        public static final double rebuffRotations = -30;
        public static final double rebuffDuration = 1;
        public static final double rebuffSpeed = 1;
    }

    public static final class Joysticks {
        // ++ controller IDs 
        public static final int primaryControllerID =0;
        public static final int secondaryControllerID = 1; 

        // ++ joystick axis IDs 
        public static final int LeftJoystickX = 2;
        public static final int LeftJoystickY = 1;
        public static final int RightJoystickX = 3;
        public static final int RightJoystickY = 4;

        // ++ joystick button IDs
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

        // ++ other joystick constants
        public static final double deadZoneSize = 0.05;

        // ++ joystick curve constants
        public static final int firstPower = 5;
        public static final int secondPower = 3; 
        public static double aCoeff = 0.1;
        public static double bCoeff = (1.0 - aCoeff); 

    }

}
