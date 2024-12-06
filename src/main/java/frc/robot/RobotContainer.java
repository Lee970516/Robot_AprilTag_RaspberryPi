// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.AprilTag;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  // private final AMPBarSubsystem m_AMPBarSubsystem = new AMPBarSubsystem();
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem = new PhotonVisionSubsystem() ;

  private final CommandXboxController driverController = new CommandXboxController(RobotContainerConstants.driverXboxController_ID);
  private final SendableChooser<Command> autoChooser;



  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    BooleanSupplier isSlow = ()-> driverController.getHID().getLeftTriggerAxis()>0.4;
    DoubleSupplier xSpeed = ()-> -driverController.getRawAxis(1);
    DoubleSupplier ySpeed = ()-> -driverController.getRawAxis(0);
    DoubleSupplier zSpeed = ()-> -driverController.getRawAxis(4);

    // driverController.x().whileTrue(new TrackNote_LimeLight(m_swerveSubsystem, m_LimeLightSubsystem, m_indexerSubsystem));
    // driverController.leftBumper().whileTrue(new TrackNote_LimeLight(m_SwerveSubsystem, m_LimeLightSubsystem, m_indexerSubsystem));
    driverController.x().whileTrue(new AprilTag(m_PhotonVisionSubsystem, m_SwerveSubsystem));
    driverController.b().whileTrue(
      Commands.runOnce(()-> {
        m_SwerveSubsystem.resetGyro();
      })
    );
    m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeed, ySpeed, zSpeed, isSlow));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    return autoChooser.getSelected();
  }
}
