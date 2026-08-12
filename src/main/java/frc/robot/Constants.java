// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public enum HubSide {
    BLUE,
    RED
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;
  }

  public static class MechanismConstants {
    public static final int kIntakePivotLeaderId = 34;
    public static final int kIntakePivotFollowerId = 35;
    public static final double kIntakePivotGearRatio = 81.0;

    public static final double kIntakePivot_kP = 20.0;
    public static final double kIntakePivot_kI = 0.0;
    public static final double kIntakePivot_kD = 0.05;
    public static final double kIntakePivot_kS = 0.0;
    public static final double kIntakePivot_kV = 0.0;
    public static final double kIntakePivot_kA = 0.0;
    public static final double kIntakePivot_kG = 0.0;

    /** Escala -1..1 para picos de tensao do closed-loop (ver IntakePivot). */
    public static final double kIntakePivotMinOutput = -0.2;
    public static final double kIntakePivotMaxOutput = 0.2;

    public static final int kIntakePivotStatorCurrentLimit = 20;
    public static final int kIntakePivotSupplyCurrentLimit = 30;
    public static final boolean kIntakePivotEnableCurrentLimit = true;

    public static final boolean kIntakePivotLeaderInverted = false;
    public static final boolean kIntakePivotFollowerInverted = false;
    public static final boolean kIntakePivotFollowerOpposed = true;

    // Fuel Roller: 1 Motor NEO Vortex (SparkFlex)
    public static final int kIntakeRollerId = 31;

    /**
     * Posicoes em rotacoes da JUNTA (encoder motor / kIntakePivotGearRatio).
     * Projeto antigo usava ~0.05 em UP; use se o zero mecanico precisar de pequeno offset.
     */
    public static final double kIntakePivotStowJointRotations = 0.0001;
    /**
     * Posicao abaixada / coleta: ajuste fino no robô. Baseado em telemetria (~-19 rot no motor).
     */
    public static final double kIntakePivotDownJointRotations = -22.2 / kIntakePivotGearRatio;

    /** Oscilacao modo tiro: dois pontos entre stow (0) e coleta (negativo). */
    public static final double kIntakePivotShootOscJointA = -0.06;
    public static final double kIntakePivotShootOscJointB = -0.10;
    /** Metade do periodo de oscilacao (s) em cada alvo. */
    public static final double kIntakeShootOscHalfPeriodSeconds = 0.7;

    public static final double kIntakeRollerCollectPower = 0.35;
    public static final double kIntakeRollerExpelPower = -0.35;

    // ---- CENTRIFUGA ----
    // 1 Motor Kraken X44
    public static final int kCentrifugeId = 36;
    public static final double kCentrifugeGearRatio = 5.0;

    // ---- FEEDER ----
    // 1 Motor NEO Vortex (SparkFlex)
    public static final int kFeederId = 30;

    // ---- SHOOTER TURRET ----
    // 1 Motor Kraken X44
    public static final int kShooterTurretId = 32;
    // Ratio a ser editado: 9 (Caixa) * (90/30 Engrenagem) = 27
    public static final double kShooterTurretGearRatio = 27.0;
    public static final double kShooterTurret_kP = 4.0; // P um pouco mais alto para auto-aim
    public static final double kShooterTurret_kI = 0.0;
    public static final double kShooterTurret_kD = 0.05;

    // Coordenadas Reais do Hub (Off-Season 2026)
    public static final Translation2d kBlueHubPose = new Translation2d(4.632, 4.035);
    public static final Translation2d kRedHubPose = new Translation2d(11.939, 4.025);

    /**
     * The current practice field uses the red Hub (AprilTags 9 and 10). Disable this on a full
     * field so targeting follows the Driver Station alliance again.
     */
    public static final boolean kUsePracticeHubOverride = true;
    public static final HubSide kPracticeHubSide = HubSide.RED;

    // ---- MODO DE PASSE (SHUTTLING/PASSING) ----
    public static final Translation2d kBlueShuttleTargetPose = new Translation2d(1.44, 1.06);
    public static final double kShuttleRPM = 3000.0;

    // ---- SHOOTER FLYWHEELS ----
    // 2 Motores NEO Vortex (SparkFlex)
    public static final int kShooterLeftId = 33;
    public static final int kShooterRightId = 37;
    public static final double kShooterFlywheels_kP = 0.0001; // REV SparkMax/Flex P
    public static final double kShooterFlywheels_kI = 0.0;
    public static final double kShooterFlywheels_kD = 0.0;
    public static final double kShooterFlywheels_kFF = 0.0018; // Ajustado para Voltagem (0.00015 * 12)
    // Velocidade Física da Nota (m/s). Use para cálculo de Tiro em Movimento (Shoot-On-The-Fly)
    public static final double kShooterNoteSpeedMetersPerSecond = 4.0;

    // ---- ELEVATOR ----
    // 1 Motor Kraken X60
    public static final int kElevatorId = 18;
    // Ratio a ser editado
    public static final double kElevatorGearRatio = 10.0;
  }

}
