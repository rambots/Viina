package com.rambots4571.chargedup.robot.utils;

import com.rambots4571.chargedup.robot.Constants.Settings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision implements Sendable {

  private final PhotonCamera camera = new PhotonCamera("Photon Cam");

  private PhotonPipelineResult result;

  private static PhotonVision instance = new PhotonVision();

  private PhotonVision() {
    SmartDashboard.putData(this);
  }

  public static PhotonVision getInstance() {
    return instance;
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public boolean hasValidTarget() {
    return result.hasTargets();
  }

  public List<PhotonTrackedTarget> getTargets() {
    return result.getTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    return result.getBestTarget();
  }

  public double getEstimatedDistanceToTarget() {
    double estimatedDistance =
        PhotonUtils.calculateDistanceToTargetMeters(
            Settings.CAMERA_HEIGHT_METERS,
            Settings.TARGET_HEIGHT_METERS,
            Settings.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(getBestTarget().getPitch()));

    return estimatedDistance;
  }

  public Sendable getTargetSendable(PhotonTrackedTarget target) {
    return builder -> {
      builder.addDoubleProperty("Yaw", target::getYaw, null);
      builder.addDoubleProperty("Pitch", target::getPitch, null);
      builder.addDoubleProperty("Area", target::getArea, null);
      builder.addDoubleProperty("Skew", target::getSkew, null);
      builder.addIntegerProperty("Tag ID", target::getFiducialId, null);
    };
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PhotonVision Values");
    builder.addBooleanProperty("Has Valid Targets", this::hasValidTarget, null);
    builder.addDoubleProperty("Estimated Distance", this::getEstimatedDistanceToTarget, null);
    getTargetSendable(getBestTarget()).initSendable(builder);
  }
}