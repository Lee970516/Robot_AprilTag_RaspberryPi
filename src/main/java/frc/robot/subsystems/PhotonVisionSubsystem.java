// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonvisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  private final PhotonCamera photonCamera;
  private final Transform3d robotToCamera;

  private final PIDController xPidController;
  private final PIDController yPidController;
  private final PIDController zPidController;

  private double xPidOutPut;
  private double yPidOutPut;
  private double zPidOutPut;


  private DoubleArraySubscriber botPose3d;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private int target_ID;

  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator poseEstimator;
  private PoseStrategy poseStrategy;


  

  
  public PhotonVisionSubsystem() {
    photonCamera = new PhotonCamera("Logitech");
    robotToCamera = new Transform3d(new Translation3d(xPidOutPut, yPidOutPut, zPidOutPut), new Rotation3d(yPidOutPut, xPidOutPut, target_ID));
    poseStrategy = PoseStrategy.CLOSEST_TO_LAST_POSE;


    xPidController = new PIDController(PhotonvisionConstants.xPid_Kp, PhotonvisionConstants.xPid_Ki, PhotonvisionConstants.xPid_Kd);
    yPidController = new PIDController(PhotonvisionConstants.yPid_Kp, PhotonvisionConstants.yPid_Ki, PhotonvisionConstants.yPid_Kd);
    zPidController = new PIDController(PhotonvisionConstants.zPid_Kp, PhotonvisionConstants.zPid_Ki, PhotonvisionConstants.zPid_Kd);

    fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    poseEstimator = new PhotonPoseEstimator(fieldLayout, poseStrategy, photonCamera, robotToCamera);
  }

  public int getTargetID() {
    return target_ID;
  }

  public Transform3d getTargetPose() {
    return target.getBestCameraToTarget();
  }

  public boolean hasTarget() {
    return result.hasTargets();
  }

  public double getXPidOutPut() {
    return xPidOutPut;
  }

  public double getYPidOutPut() {
    return yPidOutPut;
  }

  public double getZPidOutPut() {
    return zPidOutPut;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = photonCamera.getLatestResult();
    target = result.getBestTarget();
    SmartDashboard.putBoolean("Photon/hasTarget", result.hasTargets());
    if(hasTarget()){
      double botXValue = getTargetPose().getX();
      double botYValue = getTargetPose().getY();
      double botZValue = -Math.toDegrees(target.getBestCameraToTarget().getRotation().getAngle());
      target_ID = target.getFiducialId();
      SmartDashboard.putNumber("Photon/TargetID", getTargetID());
      SmartDashboard.putNumber("Photon/botXValue", botXValue);
      SmartDashboard.putNumber("Photon/botYValue", botYValue);
      SmartDashboard.putNumber("Photon/botZValue", botZValue);

      xPidOutPut = xPidController.calculate(botXValue, 0);
      yPidOutPut = yPidController.calculate(botYValue, 0);
      zPidOutPut = zPidController.calculate(botZValue, 0);
    }else {
      xPidOutPut = 0;
      yPidOutPut = 0;
      zPidOutPut = 0;
    }

  }
}
