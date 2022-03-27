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
        // ~~ Speed error threshold for crash detection
        public static final double speedErrorThreshold = 0.1;

        // ++ Motor conversion ratio stuff ------------------------------
        /**  ++ maximum RPM of the drivetrain NEOs (also the conversion factor from joystick input to RPM) */
        public static final double maxNEORPM = 5500.0;
        // ~~ Conversion ratios for drivetrain encoders
            /** ++ (velocity conversion) converts from RPM to meters per second, including gearboxes*/
        public static final double velocityConversionRatio = ((wheelDiameter * Math.PI)/(10.71 * 60));
            /** (position conversion) is the same as velocity conversion but has a cursed coefficient for some reason,,,
             * figure out what's going on with that constant */
        public static final double positionConversionRation = ((2.4 * wheelDiameter * Math.PI)/(4 * 10.71));
        /**  maximum speed of robot in m/s (max rpm times conversion ratio), this also (I think) converts from RPM to m/s */
        public static final double maxWheelSpeed = maxNEORPM * velocityConversionRatio;
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
        public static final double acceptSpeed = 1;
        public static final double acceptDuration = 0.2;
        public static final double rebuffRotations = -30;
        public static final double rebuffSpeed = -1;
        public static final double rebuffDuration = 0.2;
        // ££ constants for intakeIndexer, everything to do with the Intake/Indexer
        public static final double intakeSpeed = 0.75;
        public static final double indexSpeed = -0.75;
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
        /**  ++ lowPassFilterStrength should be between 0 & 1. The closer it is to 1, the smoother acceleration will be. */
        public static final double driveLowPassFilterStrength = 0.91;
        public static final double rotationLowPassFilterStrength = 0.2;
        // ++ we probably don't want the speed dampers as finals incase we want a fastmode/to change them later 
        public static final double driveSpeedDamper = 0.65; 
        public static final double rotationDamper = 2.0; 

        // ss This is the multiplier for Fast Mode
        // explained in JoyUtil.java
        public static final double fastModeMaxMultiplier = 0.3;


        // ++ JOYSTICK CURVE CONSTANTS --------------------------------------------------------------
        public static final double aCoeff = 0.7;
        public static final int firstPower = 3;

        public static final int secondPower = 1; 
        public static final double bCoeff = (1.0 - aCoeff); 

    }


    public static final class Shooter {
        // ++ shooter constants
        // CAN
        // ++ motor stuff ----------------------------------
        public static final int flywheelMotorID = 9;
        public static final int hoodMotorID = 5;

        // ++ PID stuff ----
        // ++ flywheel
        public static final double flywheelPGain = 0.0000015;
        public static final double flywheelIGain = 0.0025;
        public static final double flywheelDGain = 0.0;
        // ++ hood 
        public static final double hoodPGain = 0.0;
        public static final double hoodIGain = 0.0;
        public static final double hoodDGain = 0.0;
        
        // ++ ENCODER STUFF -------
        /**
         * ++ this is (maybe) the conversion ratio between the angle of the hood motor and the actual angle of the hood
         * [this ratio needs to be verified]
         */
        public static final double motorToHoodAngle = 768/7;

    }

    /**  ++ constants for limelight stuff, anything involved with calculations or keys etc */
    public static final class Limelight {
        // ++ ====== actual limelight values ============
            /** angle of the limelight; degrees up from horizontal */
        public static final double limelightAngleOffset = 25.0;

        // ++ ======= field/robot measurements ============ (all in feet)

        public static final double hubHeight = 8.6666666667; // ++ i put this as a decimal approx bc (104/12) was being weird

            /** ++ this is the height of the shooter (where the ball leaves the robot) off the ground */
        public static final double shooterHeight = 0.0; // ++ measure what this actually si

            /** this is the difference in height between the shooter and the hub, in feet */
        public static final double shooterToHubHeight = hubHeight - shooterHeight;

        // ++ these are the offsets of the arbitrary point above the ring relative to the center of the ring 
            /** ++ the horizontal distance between the (close) edge of the hub and the center */
        public static final double arbPointXOffset = -2.0;
            /** ++ the vertical distance between the hub and the arbitrary point */
        public static final double arbPointYOffset = 1.0;

    }

}
