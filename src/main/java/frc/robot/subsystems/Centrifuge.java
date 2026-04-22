package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;

/**
 * Subsistema da centrífuga: um TalonFX em modo brake que aciona o mecanismo de centrifugação.
 * <p>
 * Expõe controle em percentual de duty cycle ({@link #setPower}) e um comando WPILib que liga na
 * potência pedida e zera ao terminar ({@link #runCentrifugeCommand}).
 */
public class Centrifuge extends SubsystemBase {
    
    private final TalonFX motor = new TalonFX(MechanismConstants.kCentrifugeId);
    private final DutyCycleOut voltControl = new DutyCycleOut(0);

    public Centrifuge() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor.getConfigurator().apply(config);
    }

    public void setPower(double percent) {
        motor.setControl(voltControl.withOutput(percent));
    }

    public Command runCentrifugeCommand(double power) {
        return this.runEnd(
            () -> setPower(power),
            () -> setPower(0)
        );
    }
}
