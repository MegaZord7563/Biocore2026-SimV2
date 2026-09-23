package br.megazord.frc7563.util.geometry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class GeomUtil {

    public static Pose3d toPose3d(Transform3d transform3d) {
        return new Pose3d(
            transform3d.getTranslation(),
            transform3d.getRotation()
        );
    }

    private GeomUtil() {}
}