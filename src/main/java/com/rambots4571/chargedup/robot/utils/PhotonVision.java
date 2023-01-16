package com.rambots4571.chargedup.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision implements Sendable {

  private final PhotonCamera camera = new PhotonCamera("Photon Cam");

  private PhotonPipelineResult result;
  private ShuffleboardTab tab;

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



  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PhotonVision Values");
    builder.addBooleanProperty("Has Valid Targets", this::hasValidTarget, null);
  }
}
