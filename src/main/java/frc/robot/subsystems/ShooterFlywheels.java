package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;

/**
 * Subsistema das rodas de disparo (flywheels): dois Spark Flex + NEO Vortex, um por lado, em idle coast.
 * <p>
 * {@link #setPower} controla esquerda e direita separadamente (útil para inverter um lado na montagem).
 * {@link #runFlywheelsCommand} usa potências opostas (+/-) para girar os dois eixos em sentidos
 * complementares quando montados frente a frente.
 */
public class ShooterFlywheels extends SubsystemBase {
    
    // 2x Motores NEO Vortex (SparkFlex) direto no eixo
    private final SparkFlex leftMotor = new SparkFlex(MechanismConstants.kShooterLeftId, MotorType.kBrushless);
    private final SparkFlex rightMotor = new SparkFlex(MechanismConstants.kShooterRightId, MotorType.kBrushless);

    public ShooterFlywheels() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(SparkFlexConfig.IdleMode.kCoast);

        // Opcional: configurar limites de corrente
        // A inversão depende de como estão montados frente a frente
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Roda os Flywheels com dado percentual.
     * Importante: Geralmente o da esquerda/direita devem girar em direções opostas dependendo da montagem.
     */
    public void setPower(double percentLeft, double percentRight) {
        leftMotor.set(percentLeft);
        rightMotor.set(percentRight);
    }

    public Command runFlywheelsCommand(double power) {
        return this.runEnd(
            () -> setPower(power, -power), // Um roda em X o outro em -X (se não houver inversão no config)
            () -> setPower(0, 0)
        );
    }
}
