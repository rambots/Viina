package com.rambots4571.chargedup.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision implements Sendable {

  private final PhotonCamera camera = new PhotonCamera("Photon Cam");

  private PhotonPipelineResult result;

  private static PhotonVision instance = new PhotonVision();

  private PhotonVision() {
    SendableRegistry.addLW(this, "PhotonVision");
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

  public Sendable getSendable(PhotonTrackedTarget target) {
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
    getSendable(getBestTarget()).initSendable(builder);
  }
}
