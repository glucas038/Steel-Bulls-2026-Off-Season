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
        
        // 2. O Chassi já calcula a inércia e nos dá a coordenada mágica
        Translation2d virtualHub = drivetrain.getVirtualHub();

        // [NOVO] Posição real do atirador no campo (Offset de Paralaxe)
        // X = -0.15m (15cm pra trás do centro)
        // Y = 0.15m  (15cm pra esquerda do centro)
        Translation2d turretOffset = new Translation2d(-0.15, 0.15);
        
        // Translada e rotaciona o offset baseado na direção em que o chassi está olhando
        Translation2d turretFieldPosition = robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

        // 3. Vetor entre A ARMA e o Alvo Fantasma
        double deltaX = virtualHub.getX() - turretFieldPosition.getX();
        double deltaY = virtualHub.getY() - turretFieldPosition.getY();

        // 4. Calcula o ângulo puro no campo usando Arco Tangente 2
        Rotation2d fieldAngleToTarget = new Rotation2d(Math.atan2(deltaY, deltaX));
        
        // 5. Calcula o ângulo necessário deduzindo para onde a lataria do robô já está apontada.
        // Como o ZERO FISICO da torreta e virado para TRAS, nos subtraimos 180 graus!
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d turretAngleRelative = fieldAngleToTarget.minus(robotHeading).minus(Rotation2d.fromDegrees(180));

        double degreesNeeded = turretAngleRelative.getDegrees();
        double clampedDegrees = MathUtil.clamp(degreesNeeded, -90.0, 90.0);
        
        // ==========================================================
        // TELEMETRIA AVANÇADA PARA O ELASTIC / ADVANTAGESCOPE
        // ==========================================================
        // 1. Posições no Campo (Mostra as bolinhas no Field2d do Elastic)
        SmartDashboard.putNumberArray("Turret Debug/Virtual Hub Pose", new double[]{virtualHub.getX(), virtualHub.getY(), 0.0});
        SmartDashboard.putNumberArray("Turret Debug/Turret Field Pose", new double[]{turretFieldPosition.getX(), turretFieldPosition.getY(), robotHeading.getDegrees()});
        
        // 2. Matemática Interna
        SmartDashboard.putNumber("Turret Debug/Field Angle To Target", fieldAngleToTarget.getDegrees());
        SmartDashboard.putNumber("Turret Debug/Robot Heading", robotHeading.getDegrees());
        SmartDashboard.putNumber("Turret Debug/Turret Angle Relative (No Clamp)", turretAngleRelative.getDegrees());
        
        // 3. Resultado Final e Erro
        SmartDashboard.putNumber("Turret Debug/Clamped Target (O que o motor tenta)", clampedDegrees);
        SmartDashboard.putNumber("Turret Debug/Current Turret Angle", turret.getAngleDegrees());
        SmartDashboard.putNumber("Turret Debug/Error", clampedDegrees - turret.getAngleDegrees());

        // Envia pro PID rodar o Kraken
        turret.setTargetAngle(clampedDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        // Ao interromper/desligar a mira inteligente, solta a força do motor para deixá-lo relaxado.
        turret.setPower(0);
    }
}
