// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {
  private final SparkMax driveMotor;
  private final SparkMax rotationMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder rotationEncoder;

  private final CANcoder canCoder;
  private final double canCoderOffsetRadians;

  private final PIDController rotationPIDController;

  // PID constants
  private static final double kP = 0.25;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kTolerance = 0.01;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveID, int rotationID, int canCoderID, double canCoderOffsetRadians, boolean isDriveInverted) {
    driveMotor = new SparkMax(driveID, MotorType.kBrushless);
    rotationMotor = new SparkMax(rotationID, MotorType.kBrushless);

    // Configure drive motor
    configureMotor(driveMotor, isDriveInverted, IdleMode.kBrake, 
                   DriveTrainConstants.driveEncoderPositionConversionFactor, 
                   DriveTrainConstants.driveEncoderVelocityConversionFactor);

    // Configure rotation motor
    configureMotor(rotationMotor, true, IdleMode.kBrake, 
                   DriveTrainConstants.rotationEncoderPositionConversionFactor, 
                   DriveTrainConstants.rotationEncoderVelocityConversionFactor);

    // PID
    rotationPIDController = new PIDController(kP, kI, kD);
    rotationPIDController.setTolerance(kTolerance);
    rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Initializations
    canCoder = new CANcoder(canCoderID);
    this.canCoderOffsetRadians = canCoderOffsetRadians;
    driveEncoder = driveMotor.getEncoder();
    rotationEncoder = rotationMotor.getEncoder();
  }

  // Method to configure motor
  private void configureMotor(
    SparkMax motor, 
    boolean isInverted, 
    IdleMode idleMode, 
    double positionConversionFactor, 
    double velocityConversionFactor) {
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .inverted(isInverted)
      .idleMode(idleMode);
    config.encoder
      .positionConversionFactor(positionConversionFactor)
      .velocityConversionFactor(velocityConversionFactor);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getCANCoderRad() {
    double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
    double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRadians;
    return angle % (2 * Math.PI);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    rotationEncoder.setPosition(getCANCoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRad()));
  }

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < 0.0001) {
      stop();
      return;
    }
    
    state = optimize(state, getState().angle);

    // Set the rotation motor speed using the PID controller
    rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), state.angle.getRadians()));

    // Set the drive motor voltage using feedforward calculation
    driveMotor.setVoltage(DriveTrainConstants.driveFF.calculate(state.speedMetersPerSecond));
  }

  // optimize function to prevent motor from turning more than 90 degrees
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);

    if (Math.abs(delta.getDegrees()) > 90) {
      return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  /** Stops the swerve module. */
  public void stop() {
    driveMotor.stopMotor();
    rotationMotor.stopMotor();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getRotationPosition() {
    return rotationEncoder.getPosition();
  }

  public void initRotationOffset() {
    rotationEncoder.setPosition(getCANCoderRad());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
