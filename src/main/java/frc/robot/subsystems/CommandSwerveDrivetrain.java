package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.HubSide;
import frc.robot.Constants.MechanismConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
/**
 * Classe que estende a SwerveDrivetrain original gerada pelo Phoenix 6 e implementa
 * Subsystem do WPILib. Isso permite que ela seja usada na arquitetura Command-Based (com o método periodic(), comandos, etc).
 *
 * Gerada pelo Phoenix Tuner X e modificada por nós para adicionar funcionalidades extras
 * como o AutoBuilder do PathPlanner (em breve) e integração com o WPILib Command.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004; // 4 ms
    private static final String kLimelightName = "limelight-rear";
    private static final double kMaxVisionTagDistanceMeters = 4.0;
    private static final int kMinVisionSeedTagCount = 2;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    /* Prevent repeated pose resets after a rear-facing camera first gains sight of the Hub. */
    private boolean m_hasSeededPoseWhileEnabled = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /* Pedido vetorial para o PathPlanner (controla por velocidades em rad/s) */
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();


    private void configurePathPlanner() {

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            () -> this.getState().Pose, // Leitura da Odometria Atual
            this::resetPose,            // Zerar a Odometria
            () -> this.getState().Speeds, // Leitura da Velocidade Vetorial Atual
            (speeds, feedforwards) -> this.setControl(autoRequest.withSpeeds(speeds)), // Aplica as velocidades no Chassi
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // PID de Translação (X e Y)
                new PIDConstants(5.0, 0.0, 0.0)  // PID de Rotação (Ângulo)
            ),
            config,
            () -> {
                // Inverter o espelho dependendo se formos Aliança Vermelha ou Azul
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Requer o uso da base Drivetrain
        );
    }


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // ========================================================
        // ATUALIZAÇÃO DA ODOMETRIA USANDO A LIMELIGHT (MEGATAG 2)
        // ========================================================
        
        // The rear camera may only see the Hub after the robot turns to give it line of sight.
        // Keep applying MegaTag 1 while disabled, then allow exactly one seed after enable.
        if (DriverStation.isDisabled()) {
            m_hasSeededPoseWhileEnabled = false;
        }

        PoseEstimate seedMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightName);
        boolean seededFromVision = false;
        boolean seedMeasurementValid =
                isVisionEstimateValid(seedMeasurement, kMinVisionSeedTagCount);
        boolean canApplyVisionSeed =
                DriverStation.isDisabled() || !m_hasSeededPoseWhileEnabled;
        if (seedMeasurementValid && canApplyVisionSeed) {
            // Two tags give MegaTag 1 enough information to establish the initial field pose.
            resetPose(seedMeasurement.pose);
            seededFromVision = true;
            if (!DriverStation.isDisabled()) {
                m_hasSeededPoseWhileEnabled = true;
            }
        }

        SmartDashboard.putBoolean("Vision/Seeded from MT1", seededFromVision);
        SmartDashboard.putBoolean(
                "Vision/Waiting for Initial Seed",
                !DriverStation.isDisabled() && !m_hasSeededPoseWhileEnabled);
        SmartDashboard.putBoolean(
                "Vision/Initial Seed Complete",
                m_hasSeededPoseWhileEnabled);
        SmartDashboard.putNumber("Vision/MT1 Tag Count", seedMeasurement.tagCount);
        SmartDashboard.putNumber("Vision/MT1 Average Tag Distance", seedMeasurement.avgTagDist);

        LimelightHelpers.SetRobotOrientation(
            kLimelightName,
            this.getState().Pose.getRotation().getDegrees(), 
            0, 0, 0, 0, 0
        );

        // The camera's rear-facing 180 degree mount is configured in the Limelight web interface.
        PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        boolean visionAccepted = isVisionEstimateValid(limelightMeasurement, 1);
        if (visionAccepted) {
            // After the one-time MegaTag 1 seed, trust the drivetrain gyro for heading updates.
            this.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds,
                VecBuilder.fill(0.7, 0.7, 9999999)
            );
        }

        SmartDashboard.putBoolean("Vision/MT2 Accepted", visionAccepted);
        SmartDashboard.putNumber("Vision/MT2 Tag Count", limelightMeasurement.tagCount);
        SmartDashboard.putNumber("Vision/MT2 Average Tag Distance", limelightMeasurement.avgTagDist);

        // ========================================================
        // CALCULO DE DISTANCIA CONTINUA ATE O HUB (PARA TESTE/INTERPOLACAO)
        // ========================================================
        double distanceMeters = getHubDistanceMeters();
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Hub Distance Meters", distanceMeters);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/Shuttle Mode Active", shouldShuttle());
    }

    /**
     * Calcula a distância exata atual do robô até o Hub Fantasma (Shoot-On-The-Fly)
     * @return Distância em Metros
     */
    public double getHubDistanceMeters() {
        return this.getState().Pose.getTranslation().getDistance(getVirtualHub());
    }

    /**
     * Verifica se o robô deve entrar no Modo de Passe (Note Shuttling).
     * Ativo apenas quando na Aliança Azul e posicionado no meio da arena (X entre 5.0 e 11.5 metros).
     */
    public boolean shouldShuttle() {
        if (MechanismConstants.kUsePracticeHubOverride) {
            return false;
        }

        java.util.Optional<DriverStation.Alliance> currentAlliance = DriverStation.getAlliance();
        if (currentAlliance.isPresent() && currentAlliance.get() == DriverStation.Alliance.Blue) {
            double x = this.getState().Pose.getX();
            return x >= 5.0 && x <= 11.5;
        }
        return false;
    }

    /**
     * Calcula as coordenadas do Alvo Fantasma compensando a velocidade do robô e o tempo de voo da bola.
     * @return Coordenadas (Translation2d) do alvo virtual no campo.
     */
    public edu.wpi.first.math.geometry.Translation2d getVirtualHub() {
        edu.wpi.first.math.geometry.Translation2d targetPose;
        HubSide targetHubSide = getTargetHubSide();
        
        if (shouldShuttle()) {
            // Modo de passe dinâmico para a aliança azul
            targetPose = MechanismConstants.kBlueShuttleTargetPose;
            SmartDashboard.putString("Field/Target Hub", "SHUTTLE");
        } else {
            targetPose = targetHubSide == HubSide.RED
                    ? MechanismConstants.kRedHubPose
                    : MechanismConstants.kBlueHubPose;
            SmartDashboard.putString("Field/Target Hub", targetHubSide.name());
        }

        // Distância até o alvo para estimar o tempo de voo inicial
        double distanceToTarget = this.getState().Pose.getTranslation().getDistance(targetPose);
        
        // T = Distância / Velocidade da Bola
        double timeOfFlight = distanceToTarget / MechanismConstants.kShooterNoteSpeedMetersPerSecond;

        // Velocidades do chassi (Robot-Relative)
        double vxRobot = this.getState().Speeds.vxMetersPerSecond;
        double vyRobot = this.getState().Speeds.vyMetersPerSecond;
        double theta = this.getState().Pose.getRotation().getRadians();

        // Converte as velocidades do robô para velocidades vetoriais do campo (Field-Relative)
        double vxField = vxRobot * Math.cos(theta) - vyRobot * Math.sin(theta);
        double vyField = vxRobot * Math.sin(theta) + vyRobot * Math.cos(theta);

        // Desloca o Alvo de forma oposta à velocidade do robô multiplicada pelo tempo de voo
        double virtualX = targetPose.getX() - (vxField * timeOfFlight);
        double virtualY = targetPose.getY() - (vyField * timeOfFlight);

        return new edu.wpi.first.math.geometry.Translation2d(virtualX, virtualY);
    }

    private HubSide getTargetHubSide() {
        if (MechanismConstants.kUsePracticeHubOverride) {
            return MechanismConstants.kPracticeHubSide;
        }

        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? HubSide.RED
                : HubSide.BLUE;
    }

    private static boolean isVisionEstimateValid(PoseEstimate measurement, int minimumTagCount) {
        return measurement != null
                && measurement.tagCount >= minimumTagCount
                && measurement.avgTagDist < kMaxVisionTagDistanceMeters;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The timestamp of the pose in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    @Override
    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
    }
}
