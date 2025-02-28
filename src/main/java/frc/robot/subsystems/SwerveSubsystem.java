// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class SwerveSubsystem extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  private final SwerveModule frontLeft = new SwerveModule(
    DriveTrainConstants.frontLeftDriveMotorId, 
    DriveTrainConstants.frontLeftRotationMotorId, 
    DriveTrainConstants.frontLeftCanCoderId, 
    DriveTrainConstants.frontLeftOffsetRad,
    false);
  private final SwerveModule frontRight = new SwerveModule(
    DriveTrainConstants.frontRightDriveMotorId, 
    DriveTrainConstants.frontRightRotationMotorId, 
    DriveTrainConstants.frontRightCanCoderId, 
    DriveTrainConstants.frontRightOffsetRad,
    true);
  private final SwerveModule backLeft = new SwerveModule(
    DriveTrainConstants.backLeftDriveMotorId, 
    DriveTrainConstants.backLeftRotationMotorId, 
    DriveTrainConstants.backLeftCanCoderId, 
    DriveTrainConstants.backLeftOffsetRad,
    false);
  private final SwerveModule backRight = new SwerveModule(
    DriveTrainConstants.backRightDriveMotorId, 
    DriveTrainConstants.backRightRotationMotorId, 
    DriveTrainConstants.backRightCanCoderId, 
    DriveTrainConstants.backRightOffsetRad,
    true);

  // Create navX - Gyro
  private final AHRS navX;

  // Create swerve odometry, to figure out where the robot is on the field
  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.DriveTrainConstants.SwerveDriveKinematics, new Rotation2d(), getModulePositions());

  
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        navX.reset();
        odometry.resetPosition(new Rotation2d(), getModulePositions(), new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    // initialize the rotation offsets for the CANCoders
    frontLeft.initRotationOffset();
    frontRight.initRotationOffset();
    backLeft.initRotationOffset();
    backRight.initRotationOffset();

    // reset encoders upon each start
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getYaw());
  }

  public AHRS getNavX() {
    return navX;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getCANCoderRad())),
      new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getCANCoderRad())),
      new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getCANCoderRad())),
      new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getCANCoderRad()))
    };

    return positions;
  }

  public void setFieldRelativity() {
    if (fieldRelativeStatus) {
      fieldRelativeStatus = false;
    } else {
      fieldRelativeStatus = true;
    }
  }

  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */

    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    Logger.recordOutput("ChassisSpeed", speeds);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = DriveTrainConstants.SwerveDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.maxVelocity);

    setModuleStates(states);

  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeft.getState();
    states[1] = frontRight.getState();
    states[2] = backLeft.getState();
    states[3] = backRight.getState();

    return states;
  }



  @Override
  public void periodic() {
    odometry.update(getHeading(), getModulePositions());

    Logger.recordOutput("MyStates", getStates());

    //Logger.recordOutput("ChassisSpeeds", speeds);

    SmartDashboard.putNumber("frontLeft", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("frontRight", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("backLeft", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("backRight", frontLeft.getState().speedMetersPerSecond);
    // This method will be called once per scheduler run
  }
}
