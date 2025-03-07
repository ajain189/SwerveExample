// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  /*private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);*/

  private static final SwerveSubsystem drivetrain = new SwerveSubsystem();

  public final static Joystick logitech = new Joystick(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /*
     * Send axes and buttons from joystick to TeleopSwerveCmd,
     * which will govern the drivetrain during teleop
     */
    drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      drivetrain, 
      () -> -logitech.getRawAxis(1), // forwardX
      () -> -logitech.getRawAxis(0), // forwardY
      () -> -logitech.getRawAxis(2), // rotation
      () -> logitech.getRawAxis(3) // slider
    ));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /* BUTTON 5: RESET NAVX HEADING */
    new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
      .whileTrue(new InstantCommand(() -> drivetrain.getNavX().zeroYaw()));

    /* BUTTON 3: SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
    new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
      .whileTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));
  }
}
