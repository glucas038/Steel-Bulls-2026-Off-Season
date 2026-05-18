package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
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
    
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withSlot(0);

    public ShooterTurret() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // [TESTE FÍSICO] Invertendo a polaridade do motor devido à caixa de engrenagem
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Define coeficientes do PID posicional
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = MechanismConstants.kShooterTurret_kP;
        slot0.kI = MechanismConstants.kShooterTurret_kI;
        slot0.kD = MechanismConstants.kShooterTurret_kD;

        // Limites de velocidade para não chicotear o robô
        config.ClosedLoopGeneral.ContinuousWrap = false;

        // Motion Magic (Trapezoidal Profile) para fazer a correção ser DEVAGAR no início e suave!
        // Velocidade Máxima de cruzeiro (Rotações do Motor por segundo)
        config.MotionMagic.MotionMagicCruiseVelocity = 15.0; 
        // Aceleração da rampa (Como ela começa devagar antes de embalar)
        config.MotionMagic.MotionMagicAcceleration = 30.0;
        
        // Limita a voltagem para segurança extra (Máximo de 6V na bateria de 12V)
        config.Voltage.PeakForwardVoltage = 6;
        config.Voltage.PeakReverseVoltage = -6;

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
