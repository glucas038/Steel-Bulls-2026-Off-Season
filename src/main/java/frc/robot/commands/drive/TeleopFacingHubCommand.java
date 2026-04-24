package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

public class TeleopFacingHubCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double maxSpeed;

    private final SwerveRequest.FieldCentricFacingAngle facingAngleRequest;

    // Rampas de Aceleração em XY mais suaves (2.0 = Meio segundo pra atingir 100%)
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);

    public TeleopFacingHubCommand(
        CommandSwerveDrivetrain drivetrain, 
        DoubleSupplier forwardSupplier, 
        DoubleSupplier strafeSupplier, 
        double maxSpeed
    ) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.maxSpeed = maxSpeed;

        this.facingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity);

        // Define um PID de rotação BEM suave para não arrancar violentamente ("Na moralzinha")
        // O P original costuma ser 5+, deixamos em 2.0.
        this.facingAngleRequest.HeadingController.setPID(2.0, 0, 0.1);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // Coleta X e Y
        double rawForward = -forwardSupplier.getAsDouble();
        double rawStrafe  = -strafeSupplier.getAsDouble();

        // Limpa tremedeiras
        double deadbandForward = MathUtil.applyDeadband(rawForward, 0.1);
        double deadbandStrafe  = MathUtil.applyDeadband(rawStrafe, 0.1);

        // Suaviza nas baixas rotações e pisa fundo nas altas
        double cubedForward = Math.pow(deadbandForward, 3);
        double cubedStrafe  = Math.pow(deadbandStrafe, 3);

        // Filtro de arranque
        double limitedForward = xLimiter.calculate(cubedForward) * maxSpeed;
        double limitedStrafe  = yLimiter.calculate(cubedStrafe) * maxSpeed;

        // ============================================
        // CALCULA O ÂNGULO DA LATARIA PARA O HUB
        // ============================================
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d targetHub;
        var myAlliance = DriverStation.getAlliance();
        if (myAlliance.isPresent() && myAlliance.get() == Alliance.Red) {
            targetHub = MechanismConstants.kRedHubPose;
        } else {
            targetHub = MechanismConstants.kBlueHubPose;
        }

        double deltaX = targetHub.getX() - robotPose.getX();
        double deltaY = targetHub.getY() - robotPose.getY();

        // Calcula o ângulo base (que apontaria a FRENTE do robô para o Hub)
        Rotation2d angleToHub = new Rotation2d(Math.atan2(deltaY, deltaX));

        // Como o nosso Shooter fica na TRASEIRA, nós somamos 180 graus!
        // Assim, o robô vai dar as costas para o Hub, apontando a arma na direção exata.
        angleToHub = angleToHub.plus(Rotation2d.fromDegrees(180));

        // Aplica na CTRE o comando de Empurrar no XY do piloto e Girar a Lataria matematicamente.
        drivetrain.setControl(
            facingAngleRequest
                .withVelocityX(limitedForward)
                .withVelocityY(limitedStrafe)
                .withTargetDirection(angleToHub)
        );
    }
}
