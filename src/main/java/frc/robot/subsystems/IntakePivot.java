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
import frc.robot.Constants.MechanismConstants.IntakeArticulator;

/**
 * Intake pivot: dois TalonFX leader/follower. Config alinhada a {@link IntakeArticulator}
 * (PID, corrente, picos de tensao, inversao).
 */
public class IntakePivot extends SubsystemBase {

    private final TalonFX pivotLeader = new TalonFX(IntakeArticulator.kLeaderMotorId);
    private final TalonFX pivotFollower = new TalonFX(IntakeArticulator.kFollowerMotorId);

    private final DutyCycleOut pivotOpenLoop = new DutyCycleOut(0);
    private final PositionVoltage positionHold = new PositionVoltage(0);

    private double m_shootOscStartSec;

    public IntakePivot() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Slot0.kP = IntakeArticulator.kP;
        cfg.Slot0.kI = IntakeArticulator.kI;
        cfg.Slot0.kD = IntakeArticulator.kD;
        cfg.Slot0.kS = IntakeArticulator.kS;
        cfg.Slot0.kV = IntakeArticulator.kV;
        cfg.Slot0.kA = IntakeArticulator.kA;
        cfg.Slot0.kG = IntakeArticulator.kG;

        cfg.Voltage.PeakForwardVoltage = 12.0 * IntakeArticulator.kMaxOutput;
        cfg.Voltage.PeakReverseVoltage = 12.0 * IntakeArticulator.kMinOutput;

        if (IntakeArticulator.kEnableCurrentLimit) {
            cfg.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(IntakeArticulator.kStatorCurrentLimit))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(IntakeArticulator.kSupplyCurrentLimit))
                    .withSupplyCurrentLimitEnable(true));
        }

        cfg.MotorOutput.Inverted =
            IntakeArticulator.kLeaderInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        pivotLeader.getConfigurator().apply(cfg);

        cfg.MotorOutput.Inverted =
            IntakeArticulator.kFollowerInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        pivotFollower.getConfigurator().apply(cfg);

        MotorAlignmentValue followAlign =
            IntakeArticulator.kFollowerOpposed
                ? MotorAlignmentValue.Opposed
                : MotorAlignmentValue.Aligned;
        pivotFollower.setControl(new Follower(pivotLeader.getDeviceID(), followAlign));

        pivotLeader.setPosition(0);
    }

    /** Rotacoes da junta (encoder motor / gear ratio). */
    public double getJointPosition() {
        return pivotLeader.getPosition().getValueAsDouble() / IntakeArticulator.kGearRatio;
    }

    public void setPivotPower(double percent) {
        pivotLeader.setControl(pivotOpenLoop.withOutput(percent));
    }

    public void holdJointPosition(double jointRotations) {
        double motorRotations = jointRotations * IntakeArticulator.kGearRatio;
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
                int phase = (int) (elapsed / MechanismConstants.kIntakeShootOscHalfPeriodSeconds);
                boolean high = (phase % 2) == 0;
                holdJointPosition(
                    high
                        ? MechanismConstants.kIntakePivotShootOscJointA
                        : MechanismConstants.kIntakePivotShootOscJointB);
            },
            interrupted -> stopPivotHold(),
            () -> false,
            this)
            .withName("IntakeShootAssistOscPivot");
    }
}
