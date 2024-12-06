// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagRotation extends Command {
  /** Creates a new AprilTag. */
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;

  private double xSpeedOutPut;
  private double ySpeedOutPut;
  private double zSpeedOutPut;
  
  public AprilTagRotation(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeedOutPut = m_PhotonVisionSubsystem.getXPidOutPut();
    ySpeedOutPut = m_PhotonVisionSubsystem.getYPidOutPut();
    zSpeedOutPut = m_PhotonVisionSubsystem.getZPidOutPut();
    m_SwerveSubsystem.drive(0, 0, zSpeedOutPut, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
