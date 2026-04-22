package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterTurret;

public class AimTurretOdometryCommand extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterTurret turret;

    // Construtor
    public AimTurretOdometryCommand(CommandSwerveDrivetrain drivetrain, ShooterTurret turret) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        // Exige apenas a torreta. O Drivetrain é apenas "lido", não estamos assumindo controle de direção do volante.
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // 1. Onde está o robô agora?
        Pose2d robotPose = drivetrain.getState().Pose;
        
        // 2. Descobre quem somos (Red ou Blue) para escolher a coordenada do pino alvo
        Translation2d targetHub;
        var myAlliance = DriverStation.getAlliance();
        if (myAlliance.isPresent() && myAlliance.get() == Alliance.Red) {
            targetHub = MechanismConstants.kRedHubPose;
        } else {
            // Padrão de segurança: Blue
            targetHub = MechanismConstants.kBlueHubPose;
        }

        // 3. Vetor entre o robô e o Alvo
        double deltaX = targetHub.getX() - robotPose.getX();
        double deltaY = targetHub.getY() - robotPose.getY();

        // 4. Calcula o ângulo puro no campo usando Arco Tangente 2
        Rotation2d fieldAngleToTarget = new Rotation2d(Math.atan2(deltaY, deltaX));
        
        // 5. Calcula o ângulo necessário deduzindo para onde a lataria do robô já está apontada
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d turretAngleRelative = fieldAngleToTarget.minus(robotHeading);

        double degreesNeeded = turretAngleRelative.getDegrees();
        
        SmartDashboard.putNumber("Turret Math Needed Deg", degreesNeeded);

        // 6. Aplica a trava física de fios e limites (-90 para esquerda, 90 para direita)
        double clampedDegrees = MathUtil.clamp(degreesNeeded, -90.0, 90.0);

        // 7. Envia pro PID rodar o Kraken
        turret.setTargetAngle(clampedDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        // Ao interromper/desligar a mira inteligente, solta a força do motor para deixá-lo relaxado.
        turret.setPower(0);
    }
}
