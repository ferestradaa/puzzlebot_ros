#pragma once           // Guardia de inclusión moderna: evita que este header
                       // se incluya más de una vez en la misma unidad de compilación.
                       // Alternativa clásica: #ifndef / #define / #endif

#include <vector>      // std::vector — arreglo dinámico para los waypoints
#include <cmath>       // std::sqrt, std::cos, std::sin, std::atan2, std::abs, std::hypot
#include <limits>      // std::numeric_limits<double>::infinity()
#include <stdexcept>   // std::invalid_argument — para lanzar excepciones con mensajes útiles
#include <optional>    // std::optional — representa "puede o no haber un valor" (C++17)

// ════════════════════════════════════════════════════════════════════════════
//  TIPOS DE DATOS
//  Usamos structs (y no clases) porque son contenedores de datos puros (POD).
//  En C++ la única diferencia entre struct y class es que struct tiene
//  visibilidad pública por default. Para datos sin comportamiento, struct
//  es el idioma correcto.
// ════════════════════════════════════════════════════════════════════════════

struct Point2D {
    double x{0.0};   // Inicialización en el miembro (C++11).
    double y{0.0};   // Si construyes Point2D sin argumentos, x e y empiezan en 0.
};

// Representa el estado completo del robot en un instante dado.
// Viene de nav_msgs/Odometry en ROS2.
struct RobotState {
    double x{0.0};      // Posición global X  [m]
    double y{0.0};      // Posición global Y  [m]
    double theta{0.0};  // Orientación        [rad]  — yaw del quaternion convertido
    double v{0.0};      // Velocidad lineal   [m/s]  — de twist.linear.x
};

// Lo que el controlador devuelve cada tick. El nodo ROS lo mete en un Twist.
struct ControlOutput {
    double v{0.0};      // Velocidad lineal   [m/s]
    double omega{0.0};  // Velocidad angular  [rad/s]
};

// Todos los parámetros del controlador agrupados.
// Los valores default son un punto de partida razonable para el Puzzlebot.
struct Params {
    // ── Pure Pursuit ────────────────────────────────────────────────────────
    double ld_min{0.3};        // Lookahead mínimo [m]. No bajar de aquí aunque v → 0.
    double ld_k{0.5};          // Ganancia adaptativa: Ld = ld_k * v + ld_min
                               // A mayor velocidad, miras más lejos → más estabilidad.

    // ── Velocidad lineal ────────────────────────────────────────────────────
    double v_max{0.3};         // Velocidad máxima [m/s]
    double k_curvature{2.0};   // Frenado por curvatura: v = v_max / (1 + k_c * |κ|)
                               // Cuando la trayectoria curva, el robot frena solo.
    double a_max{0.5};         // Aceleración máxima [m/s²]. Limita saltos bruscos de v.

    // ── Corrección de cross-track error ─────────────────────────────────────
    double k_crosstrack{0.5};  // Ganancia P del error lateral.
                               // Suma un Δω proporcional a qué tan desviado está el robot
                               // de la línea del segmento activo.

    // ── Robot ────────────────────────────────────────────────────────────────
    double wheelbase{0.18};    // Distancia entre ruedas W [m]. Ajusta al URDF.

    // ── Condición de parada ──────────────────────────────────────────────────
    double goal_tol{0.12};     // Si dist(robot, último_waypoint) < goal_tol → meta. [m]
    double stop_dist{0.5};     // A partir de esta distancia al final empieza a frenar. [m]
};


// ════════════════════════════════════════════════════════════════════════════
//  CLASE PRINCIPAL
//  Sin ningún include de ROS. Puede usarse en tests unitarios, simuladores,
//  o cualquier entorno C++ sin modificación.
// ════════════════════════════════════════════════════════════════════════════

class PurePursuitController {
public:

    // ── Constructor ──────────────────────────────────────────────────────────
    // 'explicit' evita conversiones implícitas accidentales.
    // Por ejemplo, sin 'explicit' alguien podría escribir:
    //   PurePursuitController ctrl = some_params_struct;   ← confuso
    // Con 'explicit' eso no compila. Solo funciona:
    //   PurePursuitController ctrl(params);
    explicit PurePursuitController(const Params& p);

    // ── Cargar trayectoria ────────────────────────────────────────────────────
    // Recibe const ref para no copiar el vector en la llamada.
    // Internamente sí hace una copia (necesitamos ownership del path).
    void setPath(const std::vector<Point2D>& path);

    // ── Método principal ──────────────────────────────────────────────────────
    // Llamas esto en cada tick del controlador (en tu callback de odometría).
    // Recibe el estado actual del robot, devuelve v y omega listos para Twist.
    // Devuelve {0,0} si no hay path cargado o si ya se llegó a la meta.
    ControlOutput compute(const RobotState& state);

    // ── Consultas de estado ───────────────────────────────────────────────────
    bool    goalReached()        const;  // ¿Ya llegamos?
    Point2D getLookaheadPoint()  const;  // Útil para visualizar en RViz
    double  getCrossTrackError() const;  // Útil para debug y tuning
    double  getCurrentLd()       const;  // Lookahead distance actual

    // ── Reset ─────────────────────────────────────────────────────────────────
    // Limpia el estado interno sin borrar los parámetros.
    // Útil cuando recibes un path nuevo.
    void reset();

// ════════════════════════════════════════════════════════════════════════════
//  SECCIÓN PRIVADA
//  Nada de afuera puede llamar estos métodos ni acceder a estos datos.
//  Esto es encapsulamiento: el usuario de la clase no necesita saber cómo
//  funciona por dentro, solo la interfaz pública.
// ════════════════════════════════════════════════════════════════════════════
private:

    // ── Parámetros (no cambian tras construir) ───────────────────────────────
    Params p_;   // El guión bajo al final es convención para miembros privados.

    // ── Trayectoria ──────────────────────────────────────────────────────────
    std::vector<Point2D> path_;
    int    closest_idx_{0};    // Índice del segmento activo en path_
    bool   goal_reached_{false};

    // ── Estado de debug / visualización ──────────────────────────────────────
    Point2D lookahead_point_{};
    double  cross_track_error_{0.0};
    double  current_ld_{0.0};
    double  v_prev_{0.0};      // Velocidad del tick anterior (para rampa de aceleración)

    // ── Submétodos privados ───────────────────────────────────────────────────
    // Cada uno hace exactamente una cosa. compute() los orquesta.

    // Calcula Ld = ld_k * v + ld_min
    double computeLd(double v) const;

    // Avanza closest_idx_ al segmento donde el robot está actualmente.
    // Evita que el algoritmo "vuelva atrás" en la trayectoria.
    void updateClosestIdx(const RobotState& state);

    // Intersección círculo (radio=ld, centro=robot) con segmentos del path.
    // Devuelve std::optional<Point2D>:
    //   - Si encontró intersección → optional tiene valor (accedes con .value())
    //   - Si no encontró           → optional está vacío  (como un nullptr seguro)
    std::optional<Point2D> findLookahead(const RobotState& state, double ld) const;

    // Distancia perpendicular con signo del robot al segmento activo.
    // Positivo = robot está a la izquierda del segmento (mirando hacia adelante).
    // Negativo = robot está a la derecha.
    double computeCrossTrackError(const RobotState& state) const;

    // Calcula velocidad lineal deseada basada en curvatura y distancia al final.
    double computeVelocity(double kappa, const RobotState& state);
};