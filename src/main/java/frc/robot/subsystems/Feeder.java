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
 * Subsistema do alimentador (feeder): um Spark Flex com NEO Vortex brushless que empurra o projétil
 * em direção ao shooter.
 * <p>
 * Configuração é aplicada com reset seguro e persistência na flash do SPARK. Oferece
 * {@link #setPower} e {@link #runFeederCommand} para uso em comandos.
 */
public class Feeder extends SubsystemBase {
    
    private final SparkFlex motor = new SparkFlex(MechanismConstants.kFeederId, MotorType.kBrushless);

    public Feeder() {
        SparkFlexConfig config = new SparkFlexConfig();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPower(double percent) {
        motor.set(percent);
    }

    public Command runFeederCommand(double power) {
        return this.runEnd(
            () -> setPower(power),
            () -> setPower(0)
        );
    }
}
