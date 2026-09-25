package br.megazord.frc7563.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Utilitários de geometria para trabalhar com translations, rotations, transforms e poses em 2D e
 * 3D.
 */
public class GeomUtil {

  private GeomUtil() {}

  // ---------------------------------------------------------------------
  // Transform2d
  // ---------------------------------------------------------------------

  /** Cria um transform puramente translacional (2D). */
  public static Transform2d toTransform2d(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  /** Cria um transform puramente translacional (2D) a partir de x, y. */
  public static Transform2d toTransform2d(double x, double y) {
    return new Transform2d(x, y, Rotation2d.kZero);
  }

  /** Cria um transform puramente rotacional (2D). */
  public static Transform2d toTransform2d(Rotation2d rotation) {
    return new Transform2d(Translation2d.kZero, rotation);
  }

  /** Converte uma Pose2d em um Transform2d, útil em cadeias cinemáticas. */
  public static Transform2d toTransform2d(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /** Reduz um Transform3d para um Transform2d, descartando Z e as rotações fora do plano. */
  public static Transform2d toTransform2d(Transform3d transform) {
    return new Transform2d(
        transform.getTranslation().toTranslation2d(), transform.getRotation().toRotation2d());
  }

  // ---------------------------------------------------------------------
  // Pose2d
  // ---------------------------------------------------------------------

  /** Converte um Transform2d em uma Pose2d, útil como origem de uma cadeia cinemática. */
  public static Pose2d toPose2d(Transform2d transform) {
    return new Pose2d(transform.getTranslation(), transform.getRotation());
  }

  /** Cria uma pose puramente translacional (2D). */
  public static Pose2d toPose2d(Translation2d translation) {
    return new Pose2d(translation, Rotation2d.kZero);
  }

  /** Cria uma pose puramente rotacional (2D). */
  public static Pose2d toPose2d(Rotation2d rotation) {
    return new Pose2d(Translation2d.kZero, rotation);
  }

  /** Reduz uma Pose3d para uma Pose2d. */
  public static Pose2d toPose2d(Pose3d pose) {
    return new Pose2d(pose.getTranslation().toTranslation2d(), pose.getRotation().toRotation2d());
  }

  /** Retorna a inversa de uma Pose2d (equivalente a andar "para trás" pela transformação). */
  public static Pose2d inverse(Pose2d pose) {
    Rotation2d rotationInverse = pose.getRotation().unaryMinus();
    return new Pose2d(
        pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
  }

  /** Cria uma nova pose a partir de outra, trocando apenas a translação. */
  public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
    return new Pose2d(translation, pose.getRotation());
  }

  /** Cria uma nova pose a partir de outra, trocando apenas a rotação. */
  public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
    return new Pose2d(pose.getTranslation(), rotation);
  }

  // ---------------------------------------------------------------------
  // Transform3d
  // ---------------------------------------------------------------------

  /** Cria um transform puramente translacional (3D). */
  public static Transform3d toTransform3d(Translation3d translation) {
    return new Transform3d(translation, Rotation3d.kZero);
  }

  /** Cria um transform puramente rotacional (3D). */
  public static Transform3d toTransform3d(Rotation3d rotation) {
    return new Transform3d(Translation3d.kZero, rotation);
  }

  /** Converte uma Pose3d em um Transform3d, útil em cadeias cinemáticas. */
  public static Transform3d toTransform3d(Pose3d pose) {
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /** Eleva um Transform2d para um Transform3d (Z = 0, sem rotação fora do plano). */
  public static Transform3d toTransform3d(Transform2d transform) {
    return new Transform3d(
        new Translation3d(transform.getX(), transform.getY(), 0.0),
        new Rotation3d(transform.getRotation()));
  }

  // ---------------------------------------------------------------------
  // Pose3d
  // ---------------------------------------------------------------------

  /** Converte um Transform3d em uma Pose3d, útil como origem de uma cadeia cinemática. */
  public static Pose3d toPose3d(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  /** Eleva uma Pose2d para uma Pose3d (Z = 0, sem rotação fora do plano). */
  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(
        new Translation3d(pose.getX(), pose.getY(), 0.0), new Rotation3d(pose.getRotation()));
  }

  /** Cria uma pose puramente translacional (3D). */
  public static Pose3d toPose3d(Translation3d translation) {
    return new Pose3d(translation, Rotation3d.kZero);
  }

  /** Cria uma pose puramente rotacional (3D). */
  public static Pose3d toPose3d(Rotation3d rotation) {
    return new Pose3d(Translation3d.kZero, rotation);
  }

  // ---------------------------------------------------------------------
  // Twist2d / Twist3d
  // ---------------------------------------------------------------------

  /** Multiplica um twist 2D por um fator de escala. */
  public static Twist2d multiply(Twist2d twist, double factor) {
    return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
  }

  /** Multiplica um twist 3D por um fator de escala. */
  public static Twist3d multiply(Twist3d twist, double factor) {
    return new Twist3d(
        twist.dx * factor,
        twist.dy * factor,
        twist.dz * factor,
        twist.rx * factor,
        twist.ry * factor,
        twist.rz * factor);
  }

  /** Converte um ChassisSpeeds em um Twist2d (útil para integração de odometria). */
  public static Twist2d toTwist2d(ChassisSpeeds speeds) {
    return new Twist2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  // ---------------------------------------------------------------------
  // Velocidades
  // ---------------------------------------------------------------------

  /**
   * Transforma uma velocidade (ChassisSpeeds) de um ponto do robô para outro, dado o deslocamento
   * entre eles e a rotação atual do robô. Útil para calcular a velocidade de um mecanismo
   * deslocado do centro do robô (ex.: um módulo swerve ou uma câmera).
   *
   * @param velocity A velocidade original (no ponto de referência)
   * @param transform O deslocamento até a nova posição
   * @param currentRotation A rotação atual do robô
   * @return A velocidade equivalente no novo ponto
   */
  public static ChassisSpeeds transformVelocity(
      ChassisSpeeds velocity, Translation2d transform, Rotation2d currentRotation) {
    return new ChassisSpeeds(
        velocity.vxMetersPerSecond
            - velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getSin()
                    + transform.getY() * currentRotation.getCos()),
        velocity.vyMetersPerSecond
            + velocity.omegaRadiansPerSecond
                * (transform.getX() * currentRotation.getCos()
                    - transform.getY() * currentRotation.getSin()),
        velocity.omegaRadiansPerSecond);
  }
}