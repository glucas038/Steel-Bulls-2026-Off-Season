// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.commands.drive.TeleopDrive;

public class RobotContainer {
    // private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Velocidade máxima desejada do robô. Aqui limitamos a 30% (0.3) da velocidade teórica máxima a 12V.
    private double MaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    
    // Velocidade máxima de rotação do robô (giro no próprio eixo). Limitado a 1/4 de volta por segundo.
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); 

    /* Pedidos (Requests) de controle do Swerve CTRE */
    // Trava as rodas em formato de X para o robô não ser empurrado
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // Aponta as rodas para um ângulo específico sem forçar movimento
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Define o comando principal (padrão) do chassi como sendo a direção TeleopDrive.
        // A convenção WPILib dita que: Eixo X é pra frente/trás e Eixo Y é esquerda/direita.
        // Nossa classe TeleopDrive já faz a inversão necessária dos joysticks para se alinhar a isso.
        drivetrain.setDefaultCommand(
            new TeleopDrive(
                drivetrain,
                () -> joystick.getLeftY(),
                () -> joystick.getLeftX(),
                () -> joystick.getRightX(),
                MaxSpeed,
                MaxAngularRate
            )
        );

        // Quando o robô estiver "Disabled", aplica um pedido "Idle". 
        // Isso garante que os motores apliquem o seu NeutralMode (Brake ou Coast) corretamente.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Mapeamento de Botões de Ação Específica (Driver 0)
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake)); // Botão A trava as rodas em X
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            // Botão B aponta as rodas baseado no analógico esquerdo, útil para alinhar sem mover
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Rotinas do SysId (Usadas apenas na fase de tunagem/characterization)
        // Cada rotina coleta dados de física do robô para encontrar fatores de PID (Kf, Kp, Ki, Kd)
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Zera a referência de "Frente" (Field-Centric) do robô pressionando o Bumper Esquerdo.
        // Importante caso o robô perca a referência do campo ou seja posicionado errado.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Registra a função do logger (do AdvantageScope/WPILog) para capturar telemetria do chassi
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Comando autônomo temporário simples de "andar para a frente".
        // O próximo passo do projeto seria usar o PathPlanner aqui (ex: AutoBuilder.buildAuto("AutoNome")).
        final var idle = new SwerveRequest.Idle();
        final var drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

        return Commands.sequence(
            // Primeiro, garantir que a orientação Field-Centric seja tratada como "0 graus" 
            // no momento que o autônomo inicia, apontando pra longe da nossa área.
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            
            // Depois, aplica velocidade X por 5 segundos.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5) // Velocidade (m/s)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            
            // Por fim, solta os motores para ficar Idle pelo resto do autônomo.
            drivetrain.applyRequest(() -> idle)
        );
    }
}
