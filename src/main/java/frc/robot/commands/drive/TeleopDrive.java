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

    private final SwerveRequest.FieldCentric driveRequest;

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
        this.driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
            
        addRequirements(drivetrain);
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

        // 5. Envia ao chassi físico (agora multiplicando a porcentagem filtrada final pelas Velocidades Máximas)
        drivetrain.setControl(
            driveRequest.withVelocityX(limitedForward * maxSpeed)
                        .withVelocityY(limitedStrafe * maxSpeed)
                        .withRotationalRate(limitedRot * maxAngularRate)
        );
    }
}
