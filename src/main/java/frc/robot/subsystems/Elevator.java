package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

/**
 * Subsistema do elevador vertical: um TalonFX (Kraken X60) com freio ativo para não cair sob gravidade.
 * <p>
 * A posição do encoder é zerada na inicialização — o robô deve iniciar com o elevador totalmente
 * retraído. {@link #getPosition} devolve rotações do mecanismo após aplicar
 * {@link MechanismConstants#kElevatorGearRatio}.
 */
public class Elevator extends SubsystemBase {
    
    // Kraken X60 responsável pela subida/descida
    private final TalonFX motor = new TalonFX(MechanismConstants.kElevatorId);
    
    private final DutyCycleOut voltControl = new DutyCycleOut(0);

    public Elevator() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Extremamente importante que o elevador trave para não despencar pela gravidade
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motor.getConfigurator().apply(config);
        
        // Setup de Posição Inicial Sem Sensores Absolutos
        // Sempre inicie o robô com o elevador 100% retraído no chão
        motor.setPosition(0);
    }

    public void setPower(double percent) {
        motor.setControl(voltControl.withOutput(percent));
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble() / MechanismConstants.kElevatorGearRatio;
    }
}
