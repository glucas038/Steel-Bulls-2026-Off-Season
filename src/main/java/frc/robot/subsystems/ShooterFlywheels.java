package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // Controladores PID nativos dos Sparks
    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    public ShooterFlywheels() {
        SparkFlexConfig leftConfig = new SparkFlexConfig();
        SparkFlexConfig rightConfig = new SparkFlexConfig();
        
        leftConfig.idleMode(SparkFlexConfig.IdleMode.kCoast);
        rightConfig.idleMode(SparkFlexConfig.IdleMode.kCoast);

        // A inversão física foi confirmada: os motores estão de frente um para o outro
        rightConfig.inverted(true); // Se atirarem para trás na vida real, altere para o leftConfig

        // Opcional: configurar limites de corrente para não desarmar robô
        leftConfig.smartCurrentLimit(50);
        rightConfig.smartCurrentLimit(50);

        // Configuração de PID Velocity
        ClosedLoopConfig leftPID = leftConfig.closedLoop;
        ClosedLoopConfig rightPID = rightConfig.closedLoop;
        
        leftPID.p(MechanismConstants.kShooterFlywheels_kP)
               .i(MechanismConstants.kShooterFlywheels_kI)
               .d(MechanismConstants.kShooterFlywheels_kD)
               .velocityFF(MechanismConstants.kShooterFlywheels_kFF);
               
        rightPID.p(MechanismConstants.kShooterFlywheels_kP)
               .i(MechanismConstants.kShooterFlywheels_kI)
               .d(MechanismConstants.kShooterFlywheels_kD)
               .velocityFF(MechanismConstants.kShooterFlywheels_kFF);

        // Aplica os configs
        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        leftController = leftMotor.getClosedLoopController();
        rightController = rightMotor.getClosedLoopController();
    }

    /** Roda os motores via PID (RPM alvo) */
    public void setTargetRPM(double rpm) {
        // Como o direito já foi invertido no config, mandamos o número positivo pros dois!
        leftController.setReference(rpm, ControlType.kVelocity);
        rightController.setReference(rpm, ControlType.kVelocity);
        SmartDashboard.putNumber("Flywheel Target RPM", rpm);
    }

    /** Roda os motores a uma porcentagem fixa simples (Open Loop) */
    public void setPower(double percent) {
        leftMotor.set(percent);
        rightMotor.set(percent);
    }

    public Command runFlywheelsRPMCommand(double rpm) {
        return this.runEnd(
            () -> setTargetRPM(rpm),
            () -> setPower(0)
        );
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel L RPM", leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Flywheel R RPM", rightMotor.getEncoder().getVelocity());
    }
}
