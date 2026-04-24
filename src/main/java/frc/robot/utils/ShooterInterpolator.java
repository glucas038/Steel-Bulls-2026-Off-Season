package frc.robot.utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Cérebro Matemático do Atirador.
 * Guarda o mapa de relação entre Distância da Parede (Metros) e Força do Tiro (RPM).
 * O WPILib preenche as lacunas usando Regra de Três (Interpolação Linear).
 */
public class ShooterInterpolator {
    
    // O Mapa: Chave = Distância (m), Valor = RPM
    private static final InterpolatingDoubleTreeMap distanceToRpmMap = new InterpolatingDoubleTreeMap();

    static {
        // ==============================================================
        // TABELA DE CALIBRAÇÃO (PREENCHER DEPOIS DO TUNING MODE)
        // O robô aplica a curva baseada nesses dados empíricos.
        // ==============================================================
        
        // Exemplo fictício inicial:
        distanceToRpmMap.put(2.04552699677152, 2400.0);
        distanceToRpmMap.put(2.2653971029738864, 2450.0);
        distanceToRpmMap.put(2.4993061951141584, 2600.0);
        distanceToRpmMap.put(2.7595695913336287, 2800.0);
        distanceToRpmMap.put(3.012093300918247, 2950.0);
        distanceToRpmMap.put(3.277297072765206, 3100.0);
    }

    /**
     * Calcula o RPM perfeito baseado na distância atual da parede.
     * @param distanceMeters Distância calculada pela Odometria / Limelight.
     * @return O alvo em RPM para os motores aplicarem.
     */
    public static double getTargetRPM(double distanceMeters) {
        return distanceToRpmMap.get(distanceMeters);
    }
}
