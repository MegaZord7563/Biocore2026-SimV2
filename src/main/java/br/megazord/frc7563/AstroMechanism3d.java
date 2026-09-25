package br.megazord.frc7563;

import org.littletonrobotics.junction.Logger;
import br.megazord.frc7563.subsystems.shooter.ShooterConstants;
import br.megazord.frc7563.util.geometry.GeomUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;

@ExtensionMethod({GeomUtil.class})
public class AstroMechanism3d {
    private static AstroMechanism3d instance;
    
  public static AstroMechanism3d getInstance() {
    if (instance == null) {
      instance = new AstroMechanism3d();
    }
    return instance;
  }

  @Getter @Setter private Rotation2d turretAngle = Rotation2d.kZero; // Robot-relative
  @Getter @Setter private Rotation2d hoodAngle = Rotation2d.kZero; // Relative to the ground
  @Getter @Setter private double linearIntakePositionVx = 0.0;


  public void log()
  {
    var turretPose =
        ShooterConstants.Geometry.kRobotToTurret3d
            .toPose3d()
            .transformBy(
                new Transform3d(
                    Translation3d.kZero, new Rotation3d(0.0, 0.0, turretAngle.getRadians())));

    Logger.recordOutput("/Mechanism3d/Components", turretPose);
  }
}
