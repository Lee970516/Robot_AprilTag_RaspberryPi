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
  public final class SwerveConstants {
    public static final int leftFrontDriveMotorID = 29;
    public static final int rightFrontDriveMotorID = 19;
    public static final int leftBackDriveMotorID = 15;
    public static final int rightBackDriveMotorID = 16;

    public static final int leftFrontTurningMotorID = 21;
    public static final int rightFrontTurningMotorID = 17;
    public static final int leftBackTurningMotorID = 22;
    public static final int rightBackTurningMotorID = 26;

    public static final int leftFrontAbsoluteEncoderID = 43;
    public static final int rightFrontAbsoluteEncoderID = 42;
    public static final int leftBackAbsoluteEncoderID = 44;
    public static final int rightBackAbsoluteEncoderID = 41;

    public static final double leftFrontOffset = 0.4052734375;
    public static final double rightFrontOffset = 0.141845703125;
    public static final double leftBackOffset = -0.11767578125;
    public static final double rightBackOffset = 0.174072265625;

    public static final double xSpeedMaxOutPut = 0.6;
    public static final double ySpeedMaxOutPut = 0.6;
    public static final double zSpeedMaxOutPut = 0.6;
 
    public static final int pigeon2ID = 33;

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final int pidRangeMin = -180;
    public static final int pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/6.75;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double maxVelocityMetersPersecond = 5;
    public static final double maxAccelerationMeterPersecond = 10;

    public static final boolean leftFrontturningMotorInversion = true;
    public static final boolean leftFrontDriveMotorInversion = false; 
    public static final boolean leftBackturningMotorInversion = true;
    public static final boolean leftBackDriveMotorInversion = false; 
    public static final boolean rightFrontturningMotorInversion = true;
    public static final boolean rightFrontDriveMotorInversion = false; 
    public static final boolean rightBackturningMotorInversion = true;
    public static final boolean rightBackDriveMotorInversion = false; 



    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2Rad = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;

    public static final double kModuleDistance = 21*0.0254;

    public static SwerveDriveKinematics swervKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double pathingMoving_Kp = 12;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0.056;

    public static final double pathingtheta_Kp = 3;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0.035;

    public static final double maxOutput = 0;

    public static final double maxDriveMotorSpeed = 5;
    public static final double kDriveBaseRadius = 14.85 * 0.0254;

  }
  
  public final class PhotonvisionConstants {
    public static final double xPid_Kp = 0;
    public static final double xPid_Ki = 0;
    public static final double xPid_Kd = 0;

    public static final double yPid_Kp = 0;
    public static final double yPid_Ki = 0;
    public static final double yPid_Kd = 0;

    public static final double zPid_Kp = 0;
    public static final double zPid_Ki = 0;
    public static final double zPid_Kd = 0;

    public static final int blueSpeakerCenterID = 7;
    public static final int blueSpeakerLeftID = 8;
    public static final int blueAmpID = 9;
    public static final int bluesourceRightID = 1;
    public static final int blueSourceLeftID = 2;
    public static final int blueTrapLeftID = 15;
    public static final int blueTrapRightID = 16;
    public static final int blueTrapCenterID = 14; 

    public static final int redSpeakerCenterID = 4;
    public static final int redSpeakerRightID = 3;
    public static final int redAmpID = 5;
    public static final int redSourceRightID = 9;
    public static final int redSourceLeftID = 10;
    public static final int redTrapLeftID = 11;
    public static final int redTrapRightID = 12;
    public static final int redTrapCenterID = 13;
    
  }

}
