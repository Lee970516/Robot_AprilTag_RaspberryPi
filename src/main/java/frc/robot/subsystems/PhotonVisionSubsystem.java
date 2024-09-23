// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonVersion;
import org.photonvision.proto.Photon.ProtobufPhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonPipelineResultProto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonvisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  private final PhotonCamera photonCamera;

  private final PIDController xPidController;
  private final PIDController yPidController;
  private final PIDController zPidController;

  private double xPidOutPut;
  private double yPidOutPut;
  private double zPidOutPut;

  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;
  private int target_ID;

  private PhotonPoseEstimator pose;
  private AprilTagFieldLayout field;

  

  
  public PhotonVisionSubsystem() {
    photonCamera = new PhotonCamera("Logitech,_Inc._Webcam_C270");

    pose = new PhotonPoseEstimator(null, null, getTargetPose());

    field = new AprilTagFieldLayout(null, xPidOutPut, target_ID);




    xPidController = new PIDController(PhotonvisionConstants.xPid_Kp, PhotonvisionConstants.xPid_Ki, PhotonvisionConstants.xPid_Kd);
    yPidController = new PIDController(PhotonvisionConstants.yPid_Kp, PhotonvisionConstants.yPid_Ki, PhotonvisionConstants.yPid_Kd);
    zPidController = new PIDController(PhotonvisionConstants.zPid_Kp, PhotonvisionConstants.zPid_Ki, PhotonvisionConstants.zPid_Kd);
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
    var targets = result.getTargets();
    var results = result.getMultiTagResult();
    SmartDashboard.putBoolean("Photon/hasTarget", result.hasTargets());
    if(hasTarget()){
      double botXValue = getTargetPose().getX();
      double botYValue = getTargetPose().getY();
      double botZValue = -target.getYaw();
 //   double botZValue = Math.toDegrees(target.getBestCameraToTarget().getRotation().getAngle());
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
