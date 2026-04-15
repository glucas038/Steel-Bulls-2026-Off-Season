package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

/**
 * Intake pivot: dois TalonFX leader/follower.
 * (PID, corrente, picos de tensao, inversao).
 */
public class IntakePivot extends SubsystemBase {

    private final TalonFX pivotLeader = new TalonFX(MechanismConstants.kIntakePivotLeaderId);
    private final TalonFX pivotFollower = new TalonFX(MechanismConstants.kIntakePivotFollowerId);

    private final DutyCycleOut pivotOpenLoop = new DutyCycleOut(0);
    private final PositionVoltage positionHold = new PositionVoltage(0);

    private double m_shootOscStartSec;

    public IntakePivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kP = MechanismConstants.kIntakePivot_kP;
        cfg.Slot0.kI = MechanismConstants.kIntakePivot_kI;
        cfg.Slot0.kD = MechanismConstants.kIntakePivot_kD;
        cfg.Slot0.kS = MechanismConstants.kIntakePivot_kS;
        cfg.Slot0.kV = MechanismConstants.kIntakePivot_kV;
        cfg.Slot0.kA = MechanismConstants.kIntakePivot_kA;
        cfg.Slot0.kG = MechanismConstants.kIntakePivot_kG;

        cfg.Voltage.PeakForwardVoltage = 12.0 * MechanismConstants.kIntakePivotMaxOutput;
        cfg.Voltage.PeakReverseVoltage = 12.0 * MechanismConstants.kIntakePivotMinOutput;

        if (MechanismConstants.kIntakePivotEnableCurrentLimit) {
            cfg.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(MechanismConstants.kIntakePivotStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(MechanismConstants.kIntakePivotSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true));
        }

        cfg.MotorOutput.Inverted =
            MechanismConstants.kIntakePivotLeaderInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        pivotLeader.getConfigurator().apply(cfg);

        cfg.MotorOutput.Inverted =
            MechanismConstants.kIntakePivotFollowerInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        pivotFollower.getConfigurator().apply(cfg);

        MotorAlignmentValue followAlign =
            MechanismConstants.kIntakePivotFollowerOpposed
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned;
        pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), followAlign));

        pivotLeader.setPosition(0);
    }

    /** Rotacoes da junta (encoder motor / gear ratio). */
    public double getJointPosition() {
        return pivotLeader.getPosition().getValueAsDouble() / MechanismConstants.kIntakePivotGearRatio;
    }

    public void setPivotPower(double percent) {
        pivotLeader.setControl(pivotOpenLoop.withOutput(percent));
    }

    public void holdJointPosition(double jointRotations) {
        double motorRotations = jointRotations * MechanismConstants.kIntakePivotGearRatio;
        pivotLeader.setControl(positionHold.withSlot(0).withPosition(motorRotations));
    }

    public void stopPivotHold() {
        double motorRot = pivotLeader.getPosition().getValueAsDouble();
        pivotLeader.setControl(positionHold.withSlot(0).withPosition(motorRot));
    }

    public Command holdPivotToJointCommand(double jointRotations) {
        return runEnd(
            () -> holdJointPosition(jointRotations),
            this::stopPivotHold);
    }

    public Command shootAssistOscillateCommand() {
        return new FunctionalCommand(
            () -> m_shootOscStartSec = Timer.getFPGATimestamp(),
            () -> {
                double elapsed = Timer.getFPGATimestamp() - m_shootOscStartSec;

                double a = MechanismConstants.kIntakePivotShootOscJointA;
                double b = MechanismConstants.kIntakePivotShootOscJointB;

                double center = (a + b) / 2.0;
                double amplitude = Math.abs(a - b) / 2.0;
                double period = MechanismConstants.kIntakeShootOscHalfPeriodSeconds * 2.0;

                double omega = (2.0 * Math.PI) / period;
                double targetJointRotations = center + amplitude * Math.sin(omega * elapsed);

                holdJointPosition(targetJointRotations);
            },
            interrupted -> stopPivotHold(),
            () -> false,
            this
        ).withName("IntakeShootAssistOscPivot");
    }
}
