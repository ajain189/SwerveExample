// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveJoystickCmd extends Command {
  private final SwerveSubsystem drivetrain;
  private final DoubleSupplier forwardX, forwardY, rotation, slider;
  private final SlewRateLimiter xLimiter, yLimiter, rotLimiter;

  // Deadband values
  private static final double X_DEADBAND = 0.25;
  private static final double Y_DEADBAND = 0.35;
  private static final double ROT_DEADBAND = 0.4;

  /** Creates a new SwerveJoystickCmd. */
  public SwerveJoystickCmd(SwerveSubsystem drivetrain, DoubleSupplier forwardX, DoubleSupplier forwardY, DoubleSupplier rotation, DoubleSupplier slider) {

    this.drivetrain = drivetrain;
    this.forwardX = forwardX;
    this.forwardY = forwardY;
    this.rotation = rotation;
    this.slider = slider;

    // Initialize SlewRateLimiters for smooth acceleration control
    this.xLimiter = new SlewRateLimiter(DriveTrainConstants.maxAcceleration);
    this.yLimiter = new SlewRateLimiter(DriveTrainConstants.maxAcceleration);
    this.rotLimiter = new SlewRateLimiter(DriveTrainConstants.maxAngularAcceleration);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /** 
     * Units are given in meters/sec and radians/sec
     * Since joysticks give output from -1 to 1, we multiply outputs by the max speed
     * Otherwise, the max speed would be 1 m/s and 1 rad/s
     */

    // Get joystick inputs for forward, strafe, and rotation
    double xSpeed = forwardX.getAsDouble();
    double ySpeed = forwardY.getAsDouble();
    double rot = rotation.getAsDouble();

    xSpeed = applyDeadbandAndLimiter(xSpeed, X_DEADBAND, xLimiter, DriveTrainConstants.maxVelocity);
    ySpeed = applyDeadbandAndLimiter(ySpeed, Y_DEADBAND, yLimiter, DriveTrainConstants.maxVelocity);
    rot = applyDeadbandAndLimiter(rot, ROT_DEADBAND, rotLimiter, DriveTrainConstants.maxAngularVelocity);

    // Adjust speeds based on slider value
    double sliderVal = (-slider.getAsDouble() + 1) / 2;
    sliderVal = Math.max(sliderVal, 0.15);
    xSpeed *= sliderVal;
    ySpeed *= sliderVal;
    rot *= sliderVal;

    // // Apply deadband to the final speeds
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1, 1);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1, 1);
    rot = MathUtil.applyDeadband(rot, 0.3, 1);

    // Drive the robot using the calculated speeds
    drivetrain.drive(
      xSpeed,
      ySpeed, 
      rot, 
      SwerveSubsystem.fieldRelativeStatus
    );
  }
  private double applyDeadbandAndLimiter(
    double value, 
    double deadband, 
    SlewRateLimiter limiter, 
    double maxSpeed) {

    value = Math.abs(value) > deadband ? value : 0.0;
    return limiter.calculate(value) * maxSpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
