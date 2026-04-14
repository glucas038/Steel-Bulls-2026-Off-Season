// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismControllerPort = 1;
  }

  public static class MechanismConstants {
    /**
     * Intake articulator (mesma estrutura do projeto antigo). Ajuste {@link #kGearRatio} para a caixa
     * real (temporada antiga usava 45).
     */
    public static final class IntakeArticulator {

      public static final int kLeaderMotorId = 34;
      public static final int kFollowerMotorId = 35;

      public static final double kGearRatio = 81.0;

      public static final double kP = 30.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kS = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;
      public static final double kG = 0.0;

      /** Escala -1..1 para picos de tensao do closed-loop (ver IntakePivot). */
      public static final double kMinOutput = -1.0;
      public static final double kMaxOutput = 1.0;

      public static final int kStatorCurrentLimit = 20;
      public static final int kSupplyCurrentLimit = 30;
      public static final double kCurrentThresholdTime = 0.1;
      public static final boolean kEnableCurrentLimit = true;

      public static final boolean kLeaderInverted = false;
      public static final boolean kFollowerInverted = false;

      /** true = {@code Follower(..., Opposed)}. */
      public static final boolean kFollowerOpposed = true;
    }

    public static final int kIntakePivotLeaderId = IntakeArticulator.kLeaderMotorId;
    public static final int kIntakePivotFollowerId = IntakeArticulator.kFollowerMotorId;
    public static final double kIntakePivotGearRatio = IntakeArticulator.kGearRatio;

    public static final boolean kIntakePivotFollowerOpposed = IntakeArticulator.kFollowerOpposed;

    // Fuel Roller: 1 Motor NEO Vortex (SparkFlex)
    public static final int kIntakeRollerId = 31;

    /**
     * Posicoes em rotacoes da JUNTA (encoder motor / kIntakePivotGearRatio).
     * Projeto antigo usava ~0.05 em UP; use se o zero mecanico precisar de pequeno offset.
     */
    public static final double kIntakePivotStowJointRotations = 0.0;
    /**
     * Posicao abaixada / coleta: ajuste fino no robô. Baseado em telemetria (~-19 rot no motor).
     */
    public static final double kIntakePivotDownJointRotations = -19.0 / kIntakePivotGearRatio;

    /** Oscilacao modo tiro: dois pontos entre stow (0) e coleta (negativo). */
    public static final double kIntakePivotShootOscJointA = -0.12;
    public static final double kIntakePivotShootOscJointB = -0.22;
    /** Metade do periodo de oscilacao (s) em cada alvo. */
    public static final double kIntakeShootOscHalfPeriodSeconds = 0.35;

    /** Alias para codigo legado; fonte: {@link IntakeArticulator}. */
    public static final double kIntakePivot_kP = IntakeArticulator.kP;
    public static final double kIntakePivot_kI = IntakeArticulator.kI;
    public static final double kIntakePivot_kD = IntakeArticulator.kD;
    public static final double kIntakePivot_kS = IntakeArticulator.kS;
    public static final double kIntakePivot_kV = IntakeArticulator.kV;
    public static final double kIntakePivot_kA = IntakeArticulator.kA;
    public static final double kIntakePivot_kG = IntakeArticulator.kG;

    public static final double kIntakeRollerCollectPower = 0.2;
    public static final double kIntakeRollerExpelPower = -0.2;

    // ---- CENTRIFUGA ----
    // 1 Motor Kraken X44
    public static final int kCentrifugeId = 13;
    public static final double kCentrifugeGearRatio = 9.0;

    // ---- FEEDER ----
    // 1 Motor NEO Vortex (SparkFlex)
    public static final int kFeederId = 14;

    // ---- SHOOTER PIVOT ----
    // 1 Motor Kraken X44
    public static final int kShooterPivotId = 15;
    // Ratio a ser editado
    public static final double kShooterPivotGearRatio = 100.0;

    // ---- SHOOTER FLYWHEELS ----
    // 2 Motores NEO Vortex (SparkFlex)
    public static final int kShooterLeftId = 16;
    public static final int kShooterRightId = 17;

    // ---- ELEVATOR ----
    // 1 Motor Kraken X60
    public static final int kElevatorId = 18;
    // Ratio a ser editado
    public static final double kElevatorGearRatio = 10.0;
  }

}
