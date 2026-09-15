package br.megazord.frc7563.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import br.megazord.frc7563.Constants.VisionConstants;
import br.megazord.frc7563.subsystems.vision.VisionIOLimelight.AllianceOrigin;

public class VisionBuild {
    /**
   * Builds one {@link VisionCamera} per configured Limelight in {@link VisionConstants#kCameras},
   * backed by a real {@link VisionIOLimelight} each.
   */
  public static List<VisionCamera> buildRealCameras() {
    List<VisionCamera> result = new ArrayList<>();
    for (VisionConstants.CameraConfig config : VisionConstants.kCameras) {
      result.add(new VisionCamera(new VisionIOLimelight(config, AllianceOrigin.BLUE), config.name()));
    }
    return result;
  }

  /**
   * Builds one {@link VisionCamera} per configured Limelight in {@link VisionConstants#kCameras},
   * backed by a no-op {@link VisionIOSim} each - keeps the camera count/names identical between
   * SIM and REAL without simulating any hardware.
   */
  public static List<VisionCamera> buildSimCameras() {
    List<VisionCamera> result = new ArrayList<>();
    for (VisionConstants.CameraConfig config : VisionConstants.kCameras) {
      result.add(new VisionCamera(new VisionIOSim(), config.name()));
    }
    return result;
  }

}
