# 🤖 Guia de Arquitetura FRC: CTRE Phoenix Tuner Swerve

Bem-vindo(a) à FRC! O código gerado pelo **Phoenix Tuner X** para Swerve Drive possui um nível estrutural corporativo, muito robusto e otimizado pela CTRE (Cross The Road Electronics). Como esse é o seu primeiro código com o Phoenix Tuner, este guia detalha tudo que você precisa entender sobre a arquitetura do robô, os cuidados que você deve tomar e dicas fundamentais para deixar a pilotagem incisiva, mas suave.

> [!NOTE]
> O código gerado já utiliza a arquitetura **Command-Based** recomendada pela WPILib. Esse é o padrão da indústria para robôs da FRC e é essencial entender a sua separação de responsabilidades.

---

## 🏗️ 1. Arquitetura e Divisão do Código

No modelo **Command-Based**, o robô funciona em uma máquina de estados governada por *Subsistemas* (Hardware) e *Comandos* (Ações).

Você deve organizar o seu projeto dividindo as responsabilidades da seguinte forma:

### ⚙️ Subsistemas (`subsystems/`)
A representação lógica e física das partes do seu robô.
- **`CommandSwerveDrivetrain`**: Este arquivo é criado como ponte. Ele herda a infraestrutura básica do subsistema de hardware abstrato fornecido pela biblioteca nativa da CTRE e expõe métodos em forma de `Command`, permitindo interagir no padrão WPILib.
- **O que você adiciona aqui?** No futuro, quando criar um braço, um intake (coletor) ou um atirador, você criará arquivos como `Intake.java` e `Shooter.java` nesta pasta. Os subsistemas contêm apenas métodos que **atuam diretamente** nos motores e sensores.

### 🎮 Comandos (`commands/`)
Aqui fica a lógica autônoma. Um comando diz a um ou mais subsistemas *o que fazer* e *quando parar*.
- Para o Swerve no Teleop, os movimentos vindos do controle normalmente usam comandos embutidos fornecidos pelos "SwerveRequests", rodando dentro de um "Default Command" da tração.
- **O que você adiciona aqui?** Rotinas mais complexas ou sequenciais: `PegarPecaCommand`, `AtirarCommand`, ou rotinas compostas como `AtirarEPegarCommand`.

### 🧠 Centro de Controle: `RobotContainer.java`
Aqui é o "cérebro" das conexões, onde tudo se integra e ganha vida.
- Ele instancia os subsistemas.
- Inicializa os Joysticks/Controles (Xbox, PS4, etc.) do piloto.
- Configura os **botões (Triggers/Bindings)** para disparar comandos.
- Define qual é o **Comando Padrão (Default Command)** do Swerve, ou seja, como ler o joystick e mandar a informação pro chassi continuamente enquanto nada mais está sendo processado nele.

### 🔧 Constantes e Configurações (`generated/` & `Constants.java`)
- **`TunerConstants.java`**: Criado pelo gerador do Phoenix Tuner. Centraliza todos os IDs estáticos da rede CAN, ganhos do controlador PID de velocidade e posição, relações de engrenagens do hardware, e as dimensões do robô.
- **`Constants.java`**: Onde você colocará Constantes gerais do seu projeto que o Tuner desconhece (ID de sistemas paralelos, posições do braço, velocidades fixas arbitrárias do jogo).

---

## ⚠️ 2. Cuidados com o Código Gerado

O gerador do Tuner cria as rotinas de Swerve otimizadas (frequentemente usando os *Frequency-based Signals* dos motores modernos Kraken/Falcon, em conjunto com o Pigeon 2 para tempos de atualização 10x mais rápidos do que a WPILib convencional permite). Mas há ressalvas vitais:

> [!WARNING]
> **Modificações Manuais em `TunerConstants.java`**
> Não edite o arquivo `TunerConstants.java` manualmente **a menos que saiba exatamente o que mudou na matemática**. Se você perceber depois que uma roda está virando para o lado errado, e decidir usar o software Phoenix Tuner para recalibrar isso (O "Generate Project" novamente), ele **irá sobrescrever este arquivo**. Tudo que você adicionou na mão ali será deletado. Ou seja, utilize o software da CTRE para atualizar valores físicos de chassi.

> [!IMPORTANT]
> **CAN Bus**
> O gerador permite usar o CANivore (uma "placa USB" super-rápida de CAN adicional). Em seu projeto, olhe no `TunerConstants.java` ou `RobotContainer.java` qual string de CAN está sendo chamada (geralmente `"canivore"` ou `"rio"`). Se a flag selecionada no gerador não for idêntica à que fisicamente mapeia sua eletrônica no robô físico, seus equipamentos não serão detectados!

---

## 🏎️ 3. Métodos e Práticas para um Robô "Mais Suave"

Robôs de FRC equipados com o potente sistema Swerve Drive são naturalmente violentos e ágeis. Para proteger os motores (brownouts da bateria da FRC de 12V), evitar derrapagens, evitar o famoso "robô empinando/tombando", e para criar conforto no piloto para mirar com precisão, implemente os seguintes conceitos:

### A. Curvas Exponenciais (Limpar Resposta Inicial do Controle)
O analógico não é completamente linear para a percepção humana.
A fim de possuir extrema precisão nas zonas mais baixas, mas continuar atingindo a velocidade de ponta se inclinar o analógico para o topo, ajuste a escala do input.
```java
// Em vez de passar cru direto:
double forward = joystick.getLeftY();

// Suavize usando uma potência ímpa (ao cubo mantém o sinal negativo para ré):
double forwardOtimizado = Math.pow(joystick.getLeftY(), 3);
```

### B. Desativação de Zonas Mortas (Deadbands)
Mesmo solto, os polegares não centralizam perfeitamente nos potenciômetros. Existe um ruído inicial. Sempre adicione a função nativa `MathUtil.applyDeadband(valor, 0.1)` do próprio pacote da WPILib para ignorar qualquer leve tremida acidental nos analógicos.

### C. A Mágica de Slew Rate Limiters (Rampa de Aceleração)
A tática definitiva para o robô não dar "solavancos". O `SlewRateLimiter` restringe o quão repidamente uma velocidade pode ir do nível 0 à magnitude desejada (ex: em ms ou segundos).
```java
// Declarar fora do seu comando Default do Swerve:
// Esse Limiter diz que leva 1 segundo (m/s) para o valor passar de 0.0 para 1.0.
SlewRateLimiter rampaTraçãoX = new SlewRateLimiter(3.0); 

// E depois "filtrar" o valor do joytick antes de enviar para o sistema:
double forwardSuave = rampaTraçãoX.calculate(valorFiltradoJoystick);

// O mesmo pode ser feito e deve ser feito no seu giro (Rotação do chassi)
```
**OBS**: Com o CTRE Phoenix, você também pode configurar ganhos `ClosedLoopRampRate` dentro do código gerado ou nas configurações em malha fechada direto no Phoenix Tuner para limitar fisicamente as rampas internamente.

### D. Ajuste Fino via Telemetry + Tuner (SysId)
Dentro da arquitetura de vocês está incluso o `Telemetry.java`. Ele empacota toda a comunicação Swerve com as Dashboards da FRC.
Enquanto testam a suavidade:
- Usem o Phoenix Tuner na aba Swerve Generator e sigam a seção de **SysId** do Tuner ou as calibrações manuais sugeridas. Um "Gain" de PID desregulado, especialmente na direção de translação de um Kraken (Ganho de Proporcional "kP" ou Feedforward "kS"/"kV"), pode causar a "vibração" nas direções, ou transições bruscas e barulhentas.

### E. Trajetórias Fluídas e Precisas para o Autônomo
Para criar um autônomo suave, **Abandone o "Ande V Tempo, Vire X Tempo"**.
Instale a biblioteca `PathPlanner` da FRC e vincule ao `CommandSwerveDrivetrain` usando a classe nativa da PathPlanner `AutoBuilder`, ativando os Controladores PID Holonomicos Internos. Ele calcula de forma interpolada (Bézier splines) todas as curvas sem nenhuma quebra dura, gerando uma movimentação esteticamente sensacional e muito repetível.

---

## 📝 Próximos Passos Iniciais

1. Conecte ao seu ambiente, abra um terminal e rode `./gradlew build` (ou via aba no VSCode) para compilar seu código-fonte limpo.
2. Coloque seu chassi construído em cima de cadeiras ou blocos de testes (**RODAS NO AR**) sempre pela primeira vez!
3. Faça o botão de **DeployRobotCode** ou comando `Shift+F5` e assista ao Driver Station para erros em vermelho no console.
4. Experimente a direção em teleop para checar a orientação giroscópica. Certifique-se que mover o joystick para frente faz rodas girarem, e mover pra a direita de fato rotacionam todas horizontalmente para o movimento de translação correto.

Boa sorte nesta temporada da Off-Season de 2026! O domínio dessa arquitetura será uma enorme vantagem técnica da sua equipe no campo. Se quiser ajuda pra plugar o `SlewRateLimiter` ou o `Math.pow()` especificamente lendo e alterando o seu arquivo `RobotContainer.java`, é só me pedir!
