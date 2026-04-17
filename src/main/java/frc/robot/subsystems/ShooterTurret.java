package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

/**
 * Subsistema do pivô do shooter (Turret): um TalonFX (Kraken X44) com freio que define o ângulo lateral.
 * Capaz de virar 90 graus para cada lado.
 * <p>
 * O encoder é zerado na inicialização (zero mecânico ao ligar apontando pra frente).
 */
public class ShooterTurret extends SubsystemBase {
    
    // Kraken X44 responsável pelo ângulo do shooter
    private final TalonFX turretMotor = new TalonFX(MechanismConstants.kShooterTurretId);
    
    private final PositionVoltage positionControl = new PositionVoltage(0).withSlot(0);

    public ShooterTurret() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Define coeficientes do PID posicional
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = MechanismConstants.kShooterTurret_kP;
        slot0.kI = MechanismConstants.kShooterTurret_kI;
        slot0.kD = MechanismConstants.kShooterTurret_kD;

        // Limites de velocidade para não chicotear o robô
        config.ClosedLoopGeneral.ContinuousWrap = false;

        // Limita fortemente a pancada para testes: máximo 1.2V (10% de força em uma bateria 12V)
        config.Voltage.PeakForwardVoltage = 3;
        config.Voltage.PeakReverseVoltage = -3;

        turretMotor.getConfigurator().apply(config);
        
        // Setup de Posição Inicial (0 graus mecânico ao ligar)
        turretMotor.setPosition(0);
    }

    /**
     * Define o ângulo alvo da torre (Turret) usando PID interno do Kraken.
     * @param targetDegrees O ângulo em graus (-90 a 90). Onde 0 é pra frente.
     */
    public void setTargetAngle(double targetDegrees) {
        // Converte o ângulo (graus) na rotação nominal da junta e multiplica pela redução para achar a rotação do motor
        double targetRotationsAtJoint = targetDegrees / 360.0;
        double targetRotationsAtMotor = targetRotationsAtJoint * MechanismConstants.kShooterTurretGearRatio;
        
        turretMotor.setControl(positionControl.withPosition(targetRotationsAtMotor));
        SmartDashboard.putNumber("Turret Target Deg", targetDegrees);
    }

    /** Manda o motor descansar ou forçar giro manual a % */
    public void setPower(double percent) {
        turretMotor.set(percent);
    }

    /**
     * Retorna o ângulo atual do Turret em graus.
     */
    public double getAngleDegrees() {
        double motorRotations = turretMotor.getPosition().getValueAsDouble();
        double jointRotations = motorRotations / MechanismConstants.kShooterTurretGearRatio;
        return jointRotations * 360.0;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Current Deg", getAngleDegrees());
    }
}
