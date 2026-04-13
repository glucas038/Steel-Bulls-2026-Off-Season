package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

/**
 * Subsistema do pivô do shooter: um TalonFX (Kraken X44) com freio que define o ângulo de elevação.
 * <p>
 * O encoder é zerado na inicialização (zero mecânico ao ligar). {@link #getPosition} retorna
 * rotações do braço após {@link MechanismConstants#kShooterPivotGearRatio}.
 */
public class ShooterPivot extends SubsystemBase {
    
    // Kraken X44 responsável pelo ângulo do shooter
    private final TalonFX pivotMotor = new TalonFX(MechanismConstants.kShooterPivotId);
    
    private final DutyCycleOut pivotControl = new DutyCycleOut(0);

    public ShooterPivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        pivotMotor.getConfigurator().apply(config);
        
        // Setup de Posição Inicial Sem Sensores Absolutos (Liga-se no Zero)
        pivotMotor.setPosition(0);
    }

    public void setPower(double percent) {
        pivotMotor.setControl(pivotControl.withOutput(percent));
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble() / MechanismConstants.kShooterPivotGearRatio;
    }
}
