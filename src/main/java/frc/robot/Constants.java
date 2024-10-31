// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants {
 public static class OperatorConstants {
   public static final int kDriverControllerPort = 0;
 }
 public final class SwerveModConstants {
   public static final double kturingMotorP = 0.5;
   public static final double kturingMotorI = 0;//Tutorial says this is not required for the swerve mods and recommends only using P
   public static final double kturingMotorD = 0;//Same here
   public static double invertAbsoluteEncoder = 1; //Change to -1 for inversion


   public static final double kWheelDiameterMeters = Units.inchesToMeters(3.95);
   public static final double kDriveMotorGearRatio = 1/5.903;
   public static final double kTurningMotorGearRatio = 1/6.746;
   public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; //1 rotation to meters
 //DONT REALLY UNDERSTAND ABOVE AND BELOW MATH ASK FOR HELP LATER
   public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
   public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
   public static final double kTurningEncoderRPM2RADPerSec = kTurningEncoderRot2Rad / 60;  //What is this number for?
 }
 public final class DriveConstants {
   public static final double kPhysicalMaxSpeedMetersPerSecond = 4; //IDK


   //ENTER ALL THIS DATA


   //FRONT LEFT MODOOLE
   public static final int kFrontLeftDriveMotorPort = 4;
   public static final int kFrontLeftTurningMotorPort = 5;
   public static final boolean kFrontLeftDriveEncoderReversed = false;
   public static final boolean kFrontLeftTurningEncoderReversed = false;
   public static final int kFrontLeftDriveAbsoluteEncoderPort = 6;
   public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -196.606;
   public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;


   //FRONT RIGHT MODOOLE
   public static final int kFrontRightDriveMotorPort = 10;
   public static final int kFrontRightTurningMotorPort = 11;
   public static final boolean kFrontRightDriveEncoderReversed = false;
   public static final boolean kFrontRightTurningEncoderReversed = false;
   public static final int kFrontRightDriveAbsoluteEncoderPort = 12;
   public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -199.021;
   public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;


   //BACK LEFT MODOOLE
   public static final int kBackLeftDriveMotorPort = 1;
   public static final int kBackLeftTurningMotorPort = 2;
   public static final boolean kBackLeftDriveEncoderReversed = false; //THESE ARE BOTH TRUE ON OLD CODE
   public static final boolean kBackLeftTurningEncoderReversed = false;
   public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
   public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -196.272;
   public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;


   //BACK RIGHT MODOOLE
   public static final int kBackRightDriveMotorPort = 7;
   public static final int kBackRightTurningMotorPort = 8;
   public static final boolean kBackRightDriveEncoderReversed = false;
   public static final boolean kBackRightTurningEncoderReversed = false;
   public static final int kBackRightDriveAbsoluteEncoderPort = 9;
   public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -195.589;
   public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;


   //PIGEON THE SECOND
   public static final int kPigeonGyroPort = 200; //OLD CODE INVERTED THIS


   //DRIVE DEADBAND and SLEW ON STICK
   public static final double kStickDeadband =.02;
   public static final double kStickSlewLimitAccelerator = 4; //NEED TO MESS WITH THIS


   //TELEOP MAX SPEED NOT SURE ABOUT THIS ONE YET
   public static final double kMaxMetersPerSecondTeleop = 4; //I KNOW NOTHING OF IF THESE UNITS IN THE NAME ARE REAL
   public static final double kMaxRadiansPerSecondTeleop = 4;


   //KINEMATICS
   //NEEDS TUNING I DONT KNOW THESE NUMBERS
   public static final double kTrackWidth = Units.inchesToMeters(18.75); //DISTANCE BETWEEN RIGHT AND LEFT WHEELS
   public static final double kWheelBase = Units.inchesToMeters(18.75); //DISTANCE BETWEEN FRONT AND BACK WHEELS


   public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
     new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
     new Translation2d(kWheelBase / 2, kTrackWidth / 2),
     new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
     new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
   );
}
}
