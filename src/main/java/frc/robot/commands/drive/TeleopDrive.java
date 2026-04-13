package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final double maxSpeed;
    private final double maxAngularRate;

    // Modos de referência da direção:
    // FieldCentric: "Frente" no joystick leva o robô sempre para a quadra adversária, independente pra onde o bico está apontando.
    private final SwerveRequest.FieldCentric fieldCentricRequest;
    
    // RobotCentric: "Frente" no joystick leva o robô na direção em que o bico dele (chassi) está apontando.
    private final SwerveRequest.RobotCentric robotCentricRequest;
    
    // Controle interno (flag) de qual o modo ativo no momento. Inicializamos sempre na quadra (FieldCentric).
    private boolean isFieldCentric = true;

    // Filtros de Rampa de Aceleração (Slew Rate Limiters)
    // "3.0" significa que leva 1/3 (0.33) segundos para ir de 0 a 100% de velocidade.
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    public TeleopDrive(
        CommandSwerveDrivetrain drivetrain, 
        DoubleSupplier forwardSupplier, 
        DoubleSupplier strafeSupplier, 
        DoubleSupplier rotationSupplier, 
        double maxSpeed, 
        double maxAngularRate
    ) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        // Limpamos o deadband da CTRE porque agora fazemos isso matematicamente no execute
        this.fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
        this.robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
            
        addRequirements(drivetrain);
    }

    /**
     * Função chamada para inverter entre dirigir referenciado na quadra ou no bico do chassi.
     */
    public void toggleFieldCentric() {
        this.isFieldCentric = !this.isFieldCentric;
    }

    @Override
    public void execute() {
        // 1. Coleta raw (Invertendo os eixos pra se alinharem ao Padrão WPILib: Joystick pra frente é negativo)
        double rawForward = -forwardSupplier.getAsDouble();
        double rawStrafe  = -strafeSupplier.getAsDouble();
        double rawRot     = -rotationSupplier.getAsDouble();

        // 2. Deadbands (Ignora qualquer tremida do dedo até 10% do analógico)
        double deadbandForward = MathUtil.applyDeadband(rawForward, 0.1);
        double deadbandStrafe  = MathUtil.applyDeadband(rawStrafe, 0.1);
        double deadbandRot     = MathUtil.applyDeadband(rawRot, 0.1);

        // 3. Curvas Exponenciais (Elevando ao cubo)
        // Isso matematicamente deixa a rampa nas velocidades pequenas enorme para precisão,
        // mas mantém a velocidade ponta (1.0 ^ 3 = 1.0)
        double cubedForward = Math.pow(deadbandForward, 3);
        double cubedStrafe  = Math.pow(deadbandStrafe, 3);
        double cubedRot     = Math.pow(deadbandRot, 3);

        // 4. Slew Rate Limiters (Rampa de Aceleração para matar arranques repentinos)
        double limitedForward = xLimiter.calculate(cubedForward);
        double limitedStrafe  = yLimiter.calculate(cubedStrafe);
        double limitedRot     = rotLimiter.calculate(cubedRot);

        // 5. Aplica as velocidades filtradas no motor de acordo com o modo de direção selecionado
        if (isFieldCentric) {
            drivetrain.setControl(
                fieldCentricRequest.withVelocityX(limitedForward * maxSpeed)
                            .withVelocityY(limitedStrafe * maxSpeed)
                            .withRotationalRate(limitedRot * maxAngularRate)
            );
        } else {
            drivetrain.setControl(
                robotCentricRequest.withVelocityX(limitedForward * maxSpeed)
                            .withVelocityY(limitedStrafe * maxSpeed)
                            .withRotationalRate(limitedRot * maxAngularRate)
            );
        }
    }
}
