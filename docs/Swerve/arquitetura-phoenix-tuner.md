# Guia de arquitetura para o codigo Phoenix Tuner

Este projeto foi gerado pelo Phoenix Tuner X para um robo FRC com swerve CTRE/Phoenix 6. Ele ja vem com uma base que funciona, mas ainda nao e uma arquitetura completa de robo de competicao. A ideia deste documento e explicar o que cada parte faz, o que vale manter separado, quais cuidados tomar e quais pontos mexer para deixar o robo mais suave.

## 1. Como ler este projeto

Arquivos principais:

- `src/main/java/frc/robot/Robot.java`: ciclo de vida do robo. Aqui o WPILib chama `robotPeriodic`, `autonomousInit`, `teleopInit`, `testInit` etc. Em um projeto command-based, ele deve ficar simples e chamar o `CommandScheduler`.
- `src/main/java/frc/robot/RobotContainer.java`: montagem do robo. Aqui ficam os subsistemas, comandos padrao, controles do joystick e escolha do autonomo.
- `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`: subsistema do swerve. Ele herda a classe de drivetrain da CTRE e adapta para o command-based do WPILib.
- `src/main/java/frc/robot/generated/TunerConstants.java`: constantes geradas pelo Phoenix Tuner. Tem IDs CAN, offsets dos CANcoders, posicao dos modulos, relacoes de engrenagem, ganhos iniciais e factory do drivetrain.
- `src/main/java/frc/robot/Telemetry.java`: publica dados do swerve em NetworkTables, SmartDashboard e SignalLogger.
- `src/main/java/frc/robot/Constants.java`: lugar para constantes do time que nao sao geradas pelo Tuner.
- `src/main/java/frc/robot/commands/Autos.java` e arquivos `Example*`: exemplos do template WPILib. Podem ser removidos quando voces criarem comandos reais.

Regra pratica: nao trate o projeto gerado como "codigo final". Trate como uma base confiavel para hardware swerve e va criando camadas mais organizadas em volta dela.

## 2. Divisao recomendada de arquitetura

Uma boa arquitetura inicial para FRC command-based:

```text
frc.robot
  Constants.java
  Robot.java
  RobotContainer.java
  Telemetry.java
  generated/
    TunerConstants.java
  subsystems/
    CommandSwerveDrivetrain.java
    Intake.java
    Shooter.java
    Elevator.java
  commands/
    drive/
      TeleopDrive.java
      DriveToPose.java
    intake/
      IntakeCommand.java
    autos/
      AutoRoutines.java
  util/
    ControllerUtil.java
    SlewRateLimiterUtil.java
```

Nao precisa criar tudo agora. Comece com o drivetrain bem organizado, depois adicione subsistemas reais conforme o robo tiver mecanismos.

## 3. Responsabilidade de cada camada

`Robot.java` deve continuar pequeno. Ele deve agendar comandos no autonomo, cancelar no teleop quando necessario e chamar `CommandScheduler.getInstance().run()` em `robotPeriodic()`. Evite colocar regra de joystick, motor ou sensor aqui.

`RobotContainer.java` deve ser o "painel de ligacao" do robo. Ele cria os subsistemas, configura botoes e define comandos padrao. No seu projeto atual, ele tambem calcula `MaxSpeed`, `MaxAngularRate` e cria o `SwerveRequest.FieldCentric`. Isso e normal no codigo gerado, mas quando o projeto crescer vale mover a logica de dirigir para um comando separado, por exemplo `commands/drive/TeleopDrive.java`.

`CommandSwerveDrivetrain.java` deve conter comportamentos proprios do drivetrain: aplicar requests CTRE, SysId, simulacao, pose, visao, reset de heading, configuracao de perspectiva por alianca. Evite colocar joystick aqui. O subsistema nao deve saber qual controle o piloto usa.

`TunerConstants.java` deve ser tratado com cuidado. Esse arquivo representa a configuracao fisica do swerve. Mexa nele quando voce souber exatamente o que esta calibrando: IDs CAN, offsets, inversoes, dimensoes, raio da roda, relacoes de engrenagem, ganhos, corrente de slip. Evite colocar constantes gerais do robo aqui.

`Constants.java` deve guardar constantes do time e do software, como porta do controle, velocidades de teste, deadbands, limites de mecanismos e nomes de cameras. Para manter organizado, use classes internas, por exemplo `OperatorConstants`, `DriveConstants`, `VisionConstants`.

`Telemetry.java` deve ser sua camada de observabilidade. Ela nao deve comandar o robo. Use para publicar pose, velocidades, estados dos modulos e dados importantes para depurar.

## 4. O que ja existe no controle do swerve

No `RobotContainer`, o comando padrao do drivetrain e:

```java
drivetrain.applyRequest(() ->
    drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
)
```

Isso significa:

- `leftY`: anda para frente/tras.
- `leftX`: anda para esquerda/direita.
- `rightX`: gira o robo.
- `FieldCentric`: o robo dirige relativo ao campo, nao relativo a frente do chassi.
- `seedFieldCentric`: recalibra a frente do controle field-centric.
- `DriveRequestType.OpenLoopVoltage`: manda controle aberto por tensao nos motores de drive. E simples e bom para comecar, mas pode ser menos preciso do que controle fechado.

Tambem existem botoes:

- `A`: segura o robo em modo `SwerveDriveBrake`.
- `B`: aponta as rodas na direcao do stick esquerdo com `PointWheelsAt`.
- `Back + Y/X` e `Start + Y/X`: rodam SysId. Use com extremo cuidado, robo levantado ou em area segura, e apenas quando a equipe entender o procedimento.
- `Left bumper`: chama `seedFieldCentric`.

## 5. Cuidados importantes com o codigo gerado

### Nao instancie o drivetrain mais de uma vez

O proprio `TunerConstants.createDrivetrain()` diz que deve ser chamado uma vez. No seu projeto isso esta correto:

```java
public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
```

Se voce criar outro drivetrain em outro lugar, pode duplicar objetos de hardware, causar conflitos de CAN e comportamento estranho.

### Proteja `TunerConstants.java`

Antes de alterar `TunerConstants.java`, confirme:

- IDs CAN de cada TalonFX, CANcoder e Pigeon2.
- Offsets absolutos dos CANcoders.
- Inversao dos motores de drive em cada lado.
- Inversao dos steer motors e encoders.
- Posicoes X/Y dos modulos em relacao ao centro do robo.
- Raio real da roda, nao apenas o nominal.
- Nome do CAN bus, principalmente se o robo usa CANivore.

Um erro pequeno nesses valores pode fazer o robo andar torto, girar quando deveria transladar ou destruir a calibracao field-centric.

### Entenda a perspectiva da alianca

Em `CommandSwerveDrivetrain.periodic()`, o codigo aplica:

```java
setOperatorPerspectiveForward(...)
```

Azul usa 0 graus e vermelho usa 180 graus. Isso faz o controle field-centric ficar natural para o piloto dependendo da alianca. O codigo so reaplica ao desabilitar, para evitar que o robo mude comportamento no meio do teste. Mantenha essa logica a menos que voce tenha uma razao clara para mudar.

### Nao misture joystick com subsistema

O drivetrain deve receber requests ou comandos, nao ler controle diretamente. Isso facilita testar, simular e criar autonomos. Se quiser melhorar a arquitetura, extraia o trecho de teleop para um comando `TeleopDrive`.

### Tome cuidado com unidades

Este projeto usa o sistema de unidades do WPILib/CTRE:

```java
MetersPerSecond.of(...)
RotationsPerSecond.of(...)
Inches.of(...)
Amps.of(...)
```

Evite misturar polegadas, metros, rotacoes e radianos como `double` solto. Sempre que possivel, mantenha o tipo com unidade ate o ultimo momento.

### Teste SysId com procedimento

Os comandos `sysIdDynamic` e `sysIdQuasistatic` movem o robo propositalmente para caracterizar ganhos. Nao rode isso perto de pessoas ou paredes. O ideal e ter:

- robo inspecionado mecanicamente;
- bateria boa;
- campo livre;
- robo com espaco para acelerar;
- pessoa pronta para desabilitar;
- logs salvos e identificados.

## 6. Como deixar o robo mais suave

A suavidade vem de uma combinacao de controle do piloto, limits de aceleracao, ganhos corretos e mecanica boa.

### 6.1 Ajuste deadband corretamente

Hoje o codigo usa:

```java
.withDeadband(MaxSpeed * 0.1)
.withRotationalDeadband(MaxAngularRate * 0.1)
```

Isso cria uma zona morta de 10%. E um bom ponto de partida, mas pode ficar "duro" se o controle for bom. Teste algo entre 5% e 10%. Deadband baixo demais causa drift. Deadband alto demais faz o robo parecer que demora para responder.

Metodo importante:

- `withDeadband(...)`
- `withRotationalDeadband(...)`

### 6.2 Use curva no joystick

Para controle mais fino em baixa velocidade, aplique curva nos eixos. Exemplo conceitual:

```java
private static double squareWithSign(double value) {
    return Math.copySign(value * value, value);
}
```

Com isso, stick em 50% vira 25% de comando, mas stick em 100% continua 100%. O piloto ganha precisao perto do centro.

Cuidados:

- aplique deadband antes ou junto da curva;
- preserve o sinal;
- nao use curva agressiva demais na rotacao se o piloto precisar virar rapido.

### 6.3 Use SlewRateLimiter

O `SlewRateLimiter` limita a velocidade com que o comando muda. Isso ajuda a evitar arrancadas bruscas:

```java
SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
SlewRateLimiter rotLimiter = new SlewRateLimiter(4.0);
```

Em seguida, voce passa o valor do joystick pelo limiter antes de multiplicar por `MaxSpeed`.

Cuidados:

- se limitar demais, o robo fica lento para reagir;
- comece com limites conservadores e teste com o piloto;
- use limites separados para translacao e rotacao.

### 6.4 Considere controle fechado no drive

Hoje o request usa:

```java
.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
```

Open-loop e simples. Para movimento mais consistente, principalmente em autonomo, voce pode avaliar `DriveRequestType.Velocity`, desde que os ganhos de drive estejam bons em `TunerConstants`.

Metodo/ponto importante:

- `withDriveRequestType(...)`
- `DriveRequestType.OpenLoopVoltage`
- `DriveRequestType.Velocity`
- `driveGains` em `TunerConstants.java`

Nao troque para velocity sem testar. Se os ganhos estiverem ruins, o robo pode oscilar ou responder pior.

### 6.5 Reduza velocidade maxima para treino

No inicio, nao use 100% da velocidade teorica. O projeto atual faz:

```java
private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
```

Para treino, voce pode usar 60% a 80%:

```java
private double MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
```

Isso nao substitui tuning, mas ajuda a aprender com mais seguranca.

### 6.6 Ajuste velocidade angular

Hoje:

```java
private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
```

Se o robo gira muito agressivo, reduza para algo como 0.5 rotacao/s durante treino. Se gira lento demais, suba aos poucos. Rotacao brusca costuma ser o que mais atrapalha piloto iniciante.

### 6.7 Faca tuning mecanico antes de tuning de software

Software nao corrige bem:

- roda com raio diferente;
- bevel gear ruim;
- modulo preso;
- correia/corrente com tensao errada;
- CANcoder com offset errado;
- roda invertida;
- Pigeon2 mal fixado;
- bateria fraca;
- robo com centro de massa muito alto.

Antes de mexer em PID, confira se todos os modulos giram livremente e apontam para o angulo correto.

## 7. Metodos e APIs que vale estudar

No Phoenix/CTRE swerve:

- `TunerConstants.createDrivetrain()`: cria o drivetrain uma unica vez.
- `drivetrain.applyRequest(...)`: transforma um `SwerveRequest` em comando WPILib.
- `setControl(...)`: aplica o request CTRE no drivetrain.
- `registerTelemetry(...)`: registra callback para receber estado do swerve.
- `seedFieldCentric()`: redefine a referencia field-centric atual.
- `seedFieldCentric(Rotation2d)`: redefine a referencia para um angulo especifico.
- `setOperatorPerspectiveForward(...)`: define o que e "frente" para o operador.
- `addVisionMeasurement(...)`: adiciona pose de camera no estimador.
- `setVisionMeasurementStdDevs(...)`: ajusta confianca da visao.
- `samplePoseAt(...)`: consulta pose em um instante, util para visao com latencia.

No `SwerveRequest`:

- `SwerveRequest.FieldCentric`: direcao baseada no campo.
- `SwerveRequest.RobotCentric`: direcao baseada na frente do robo.
- `withVelocityX(...)`
- `withVelocityY(...)`
- `withRotationalRate(...)`
- `withDeadband(...)`
- `withRotationalDeadband(...)`
- `withDriveRequestType(...)`
- `SwerveRequest.SwerveDriveBrake`
- `SwerveRequest.PointWheelsAt`
- `SwerveRequest.Idle`

No WPILib command-based:

- `setDefaultCommand(...)`
- `run(...)`
- `runOnce(...)`
- `whileTrue(...)`
- `onTrue(...)`
- `Commands.sequence(...)`
- `withTimeout(...)`
- `CommandScheduler.getInstance().run()`
- `RobotModeTriggers.disabled()`

Para suavidade:

- `SlewRateLimiter`
- `MathUtil.applyDeadband(...)`
- curvas de joystick, como quadratica ou cubica com sinal preservado;
- reducao temporaria de `MaxSpeed` e `MaxAngularRate`;
- `DriveRequestType.Velocity` depois que os ganhos estiverem validados.

## 8. Primeiro refactor recomendado

O primeiro refactor seguro e separar o comando de teleop:

1. Criar `commands/drive/TeleopDrive.java`.
2. Passar para ele o `CommandSwerveDrivetrain` e `DoubleSupplier` para `x`, `y` e `rot`.
3. Dentro do comando, aplicar deadband, curva e opcionalmente `SlewRateLimiter`.
4. Deixar `RobotContainer` apenas conectando joystick ao comando.
5. Manter `TunerConstants.java` sem mudancas nesse refactor.

Formato desejado:

```java
drivetrain.setDefaultCommand(
    new TeleopDrive(
        drivetrain,
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX(),
        () -> -joystick.getRightX()
    )
);
```

Isso deixa o `RobotContainer` mais limpo e facilita testar diferentes estilos de controle sem mexer na configuracao do hardware.

## 9. Checklist antes de colocar no robo

- Confirmar team number em `.wpilib/wpilib_preferences.json`.
- Confirmar IDs CAN no Phoenix Tuner.
- Confirmar se o CAN bus em `TunerConstants.kCANBus` esta certo.
- Conferir Pigeon2 ID e orientacao fisica.
- Conferir offsets de todos os CANcoders.
- Levantar o robo e testar cada modulo individualmente.
- Validar frente, esquerda e rotacao no field-centric.
- Testar `seedFieldCentric` com o robo apontado para a frente desejada.
- Ver no dashboard se a pose muda no sentido esperado.
- Reduzir `MaxSpeed` no primeiro teste com o robo no chao.
- Gravar logs quando fizer SysId ou tuning.

## 10. Ordem pratica de evolucao

1. Fazer o robo dirigir de forma previsivel em baixa velocidade.
2. Validar field-centric e botao de reset de heading.
3. Adicionar curva de joystick e `SlewRateLimiter`.
4. Remover comandos e subsistemas de exemplo.
5. Separar `TeleopDrive` em `commands/drive`.
6. Criar constantes organizadas em `Constants.java`.
7. Criar autonomo simples com PathPlanner ou comandos proprios.
8. Adicionar visao apenas depois que odometria e heading estiverem confiaveis.
9. Rodar SysId com procedimento seguro.
10. Ajustar ganhos e limites com base em logs, nao por tentativa cega.

## 11. Mentalidade para aprender

Sempre que for mexer no codigo, pergunte:

- Estou mexendo em configuracao fisica do swerve ou em comportamento do piloto?
- Esse valor e uma constante do robo, uma calibracao do Tuner ou uma preferencia de controle?
- Consigo testar isso com o robo suspenso antes de colocar no chao?
- Tenho log ou dashboard para confirmar o que aconteceu?
- Essa mudanca deveria ficar no subsistema, no comando ou no `RobotContainer`?

Se voce mantiver essas separacoes desde o comeco, o codigo cresce sem virar uma mistura de joystick, motor, sensor e autonomo no mesmo arquivo.
