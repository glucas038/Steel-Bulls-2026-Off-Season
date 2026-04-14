package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

/**
 * Rolete do intake (Spark Flex + NEO Vortex). Subsistema separado do pivô para permitir roller e
 * articulação em paralelo no scheduler.
 */
public class IntakeRoller extends SubsystemBase {

    private final SparkFlex roller = new SparkFlex(MechanismConstants.kIntakeRollerId, MotorType.kBrushless);

    public IntakeRoller() {
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.idleMode(SparkFlexConfig.IdleMode.kCoast);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setPower(double percent) {
        roller.set(percent);
    }

    public Command runRollerCommand(double power) {
        return runEnd(
            () -> setPower(power),
            () -> setPower(0));
    }
}
