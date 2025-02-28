// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    public static final int JoystickResetHeading = 5;
    public static final int JoystickRobotRelative = 3;
  }

  public static class DriveTrainConstants {

    public static final double wheelBase = Units.inchesToMeters(25.125); // distance between front wheels (like train track)
    public static final double trackWidth = Units.inchesToMeters(21.25); // distance from center of wheels on side

    public static final double wheelDiameter = Units.inchesToMeters(4.0 / 1.0);

    // Kinematics gets each module relative to center. X is forward/backward and Y is left/right
    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front right (+,+)
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // back right (+,-)
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // front left (-,+)
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // back left (-,-)
    );

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */

    /*
     *  Gear Ratios: found on google at YAGSL Standard Conversions
     *  (Note: Gear Ratio is the ratio of the motor to the wheel, so a gear ratio of 1:1 means that 1 motor rotation = 1 wheel rotation)
     */

    // Given Motor Rotations, convert to Meters traveled
    // (1 rev / Gear Ratio) * ((2 * PI * r) / (1 Rev)) = 
    // (2 * PI * r) / (Gear Ratio) = 
    public static final double driveEncoderPositionConversionFactor = 0.0540992905;

    // dx/dt
    // Given RPM, convert to m/s
    public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60.0;

    // Given Motor Rotations, convert to Radians travelled
    // (1 rev / Gear Ratio) * ((2 * PI) RAD / (1 Rev))
    // (2 * PI) RAD / (Gear Ratio)
    public static final double rotationEncoderPositionConversionFactor = 0.335103217;

    // Given RPM, convert to radians/seconds
    public static final double rotationEncoderVelocityConversionFactor = rotationEncoderPositionConversionFactor / 60.0;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */
    
    // Maximums
    public static final double maxVelocity = 5; // meters per second
    public static final double maxAcceleration = 10; // meters/sec^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
    // Teleop max speeds
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0; // meters/sec
    public static final double kTeleDriveMaxAngularSpeed = 3; // rad/sec

    /* ============== */
    /* SWERVE MODULES */
    /* ============== */

    /*
     * CAN IDs: found and set via REV hardware client
     * CANcoder Offsets: found in Phoenix Tuner X as "Absolute position"
     * after manually straightening wheel (converted to radians here by multiplying by 2 * PI)
     * (Note: CANcoder is mounted on the wheel, so the offset is the angle of the wheel when the robot is straight)
     */

    // front left
    public static final int frontLeftDriveMotorId = 1;
    public static final int frontLeftRotationMotorId = 2;
    public static final int frontLeftCanCoderId = 11;
    public static final double frontLeftOffsetRad = 0.867676 * 2 * Math.PI;
    // front right
    public static final int frontRightDriveMotorId = 8;
    public static final int frontRightRotationMotorId = 7;
    public static final int frontRightCanCoderId = 12;
    public static final double frontRightOffsetRad = 0.038330 * 2 * Math.PI;
    // back left
    public static final int backLeftDriveMotorId = 5;
    public static final int backLeftRotationMotorId = 6;
    public static final int backLeftCanCoderId = 14;
    public static final double backLeftOffsetRad = 0.245361 * 2 * Math.PI;
    // back right
    public static final int backRightDriveMotorId = 4;
    public static final int backRightRotationMotorId = 3;
    public static final int backRightCanCoderId = 13;
    public static final double backRightOffsetRad = 0.473633 * 2 * Math.PI;

    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */

    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.2, 2.5,0.0);

  }
}
