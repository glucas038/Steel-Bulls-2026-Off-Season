package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;

public class Intake extends SubsystemBase {
    
    // Articulação (Pivot)
    private final TalonFX pivotLeader = new TalonFX(MechanismConstants.kIntakePivotLeaderId);
    private final TalonFX pivotFollower = new TalonFX(MechanismConstants.kIntakePivotFollowerId);
    
    // Coletor de Fuel (Roller)
    private final SparkFlex roller = new SparkFlex(MechanismConstants.kIntakeRollerId, MotorType.kBrushless);

    private final DutyCycleOut pivotControl = new DutyCycleOut(0);

    public Intake() {
        // Configurações do Pivot (Krakens)
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        pivotLeader.getConfigurator().apply(pivotConfig);
        pivotFollower.getConfigurator().apply(pivotConfig);
        
        // Aligned: mesmo sentido mecânico que o líder; use Opposed se os eixos forem contrários.
        pivotFollower.setControl(
            new Follower(pivotLeader.getDeviceID(), MotorAlignmentValue.Aligned));

        // Inicializa o encoder do Pivot no hard-stop atual
        pivotLeader.setPosition(0);

        // Configurações do Roller (NEO Vortex)
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.idleMode(SparkFlexConfig.IdleMode.kCoast);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Seta a força bruta da articulação do Intake.
     * Use com muito cuidado por causa do ratio de 81:1.
     */
    public void setPivotPower(double percent) {
        pivotLeader.setControl(pivotControl.withOutput(percent));
    }

    /**
     * Seta a força bruta do rolete coletor.
     */
    public void setRollerPower(double percent) {
        roller.set(percent);
    }

    /**
     * Retorna a posição relativa do Pivot (em rotações do motor).
     * Para saber graus da junta, é necessário dividir por MechanismConstants.kIntakePivotGearRatio (81).
     */
    public double getPivotPosition() {
        return pivotLeader.getPosition().getValueAsDouble() / MechanismConstants.kIntakePivotGearRatio;
    }

    // Comandos práticos sugeridos
    public Command runRollerCommand(double power) {
        return this.runEnd(
            () -> setRollerPower(power),
            () -> setRollerPower(0)
        );
    }
}
