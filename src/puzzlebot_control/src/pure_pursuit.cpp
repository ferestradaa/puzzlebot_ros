#include "puzzlebot_control/pure_pursuit_controller.hpp"
#include <algorithm>
// ════════════════════════════════════════════════════════════════════════════
//  NOTA SOBRE SINTAXIS: scope resolution operator  "::"
//  Cuando defines fuera de la clase métodos que declaraste dentro,
//  necesitas decirle al compilador "este compute() es el de PurePursuitController,
//  no una función global". Eso se hace con  NombreClase::nombreMetodo().
// ════════════════════════════════════════════════════════════════════════════


// ── Constructor ──────────────────────────────────────────────────────────────
// La lista de inicialización  ": p_(p)"  inicializa el miembro p_ con el
// parámetro p ANTES de que el cuerpo del constructor corra.
// Es más eficiente que asignar dentro del cuerpo porque evita una construcción
// default + asignación. Para tipos simples no importa, pero es buen hábito.
PurePursuitController::PurePursuitController(const Params& p)
    : p_(p)                    // copia los parámetros al miembro p_
    , closest_idx_(0)          // empezamos desde el primer segmento
    , goal_reached_(false)
    , cross_track_error_(0.0)
    , current_ld_(p.ld_min)
    , v_prev_(0.0)
{
    // Nada más que hacer — los miembros se inicializaron arriba.
}


// ── setPath ──────────────────────────────────────────────────────────────────
void PurePursuitController::setPath(const std::vector<Point2D>& path)
{
    // Lanzamos excepción si el path no tiene sentido.
    // std::invalid_argument es la excepción estándar para parámetros inválidos.
    if (path.size() < 2) {
        throw std::invalid_argument(
            "[PurePursuit] El path necesita al menos 2 waypoints.");
    }

    path_ = path;   // Aquí sí copiamos — la clase necesita su propia copia.
    reset();        // Reinicia el estado interno para el nuevo path.
}


// ── reset ────────────────────────────────────────────────────────────────────
void PurePursuitController::reset()
{
    closest_idx_      = 0;
    goal_reached_     = false;
    cross_track_error_= 0.0;
    current_ld_       = p_.ld_min;
    v_prev_           = 0.0;
    lookahead_point_  = Point2D{};   // Point2D{} construye con x=0, y=0 (valores default)
}


// ════════════════════════════════════════════════════════════════════════════
//  compute() — EL CORAZÓN DEL CONTROLADOR
//  Se llama en cada tick (típicamente en el callback de odometría).
//  Orquesta todos los submétodos en el orden correcto del algoritmo.
// ════════════════════════════════════════════════════════════════════════════
ControlOutput PurePursuitController::compute(const RobotState& state)
{
    // ── Guardas de seguridad ──────────────────────────────────────────────────
    // Si no hay path o ya llegamos, detenemos el robot inmediatamente.
    // Retornar early (return anticipado) es más limpio que anidar todo en un if.
    if (path_.empty() || goal_reached_) {
        return ControlOutput{0.0, 0.0};
    }

    // ── PASO 1: ¿Llegamos a la meta? ─────────────────────────────────────────
    // Comprobamos distancia al ÚLTIMO waypoint del path.
    // std::hypot(dx, dy) calcula sqrt(dx² + dy²) de forma numéricamente estable.
    const Point2D& goal = path_.back();   // .back() = último elemento del vector
    const double dist_to_goal = std::hypot(goal.x - state.x, goal.y - state.y);

    if (dist_to_goal < p_.goal_tol) {
        goal_reached_ = true;
        return ControlOutput{0.0, 0.0};
    }

    // ── PASO 2: Avanzar el índice del segmento activo ─────────────────────────
    // Esto evita que el algoritmo "vuelva atrás" si el robot se desvía.
    updateClosestIdx(state);

    // ── PASO 3: Calcular Ld adaptativo ───────────────────────────────────────
    // Ld = ld_k * v_actual + ld_min
    // Cuanta más velocidad, más lejos miramos → más estabilidad.
    current_ld_ = computeLd(state.v);

    // ── PASO 4: Encontrar el lookahead point ─────────────────────────────────
    // Buscamos la intersección del círculo de radio Ld con los segmentos del path.
    // std::optional<Point2D>: puede tener un valor o estar vacío.
    std::optional<Point2D> lp = findLookahead(state, current_ld_);

    // Si no hay intersección (robot muy desviado, path terminando), usamos
    // el último waypoint como fallback para no quedarnos sin comando.
    if (!lp.has_value()) {
        lp = goal;
    }

    // Guardamos para debug/visualización
    // .value() extrae el Point2D de dentro del optional (ya sabemos que tiene valor)
    lookahead_point_ = lp.value();

    // ── PASO 5: Transformar lookahead point al marco del robot ────────────────
    // Necesitamos saber dónde está el punto desde la perspectiva del robot,
    // no en coordenadas globales.
    //
    // dx_g, dy_g: vector global del robot al lookahead point
    const double dx_g = lookahead_point_.x - state.x;
    const double dy_g = lookahead_point_.y - state.y;

    // Rotación inversa por -theta (marco local del robot):
    //   x_local =  cos(θ)*dx + sin(θ)*dy   ← componente "adelante"
    //   y_local = -sin(θ)*dx + cos(θ)*dy   ← componente "izquierda"
    //
    // No necesitamos x_local para el cálculo (solo y_local importa en Pure Pursuit),
    // pero lo dejamos comentado para referencia.
    // const double x_local =  std::cos(state.theta)*dx_g + std::sin(state.theta)*dy_g;
    const double y_local = -std::sin(state.theta)*dx_g + std::cos(state.theta)*dy_g;

    // ── PASO 6: Calcular curvatura κ ──────────────────────────────────────────
    // Esta es LA ecuación de Pure Pursuit:
    //   κ = 2 * y_local / Ld²
    //
    // Intuición:
    //   y_local = 0  → punto justo enfrente → κ = 0 → va recto
    //   y_local > 0  → punto a la izquierda → κ > 0 → gira izquierda
    //   y_local < 0  → punto a la derecha   → κ < 0 → gira derecha
    const double kappa = (2.0 * y_local) / (current_ld_ * current_ld_);

    // ── PASO 7: Cross-track error y corrección ────────────────────────────────
    // Pure Pursuit no tiene corrección explícita de posición.
    // Agregamos un término proporcional al error lateral para recuperar desviaciones.
    //
    //   ω_total = ω_pure_pursuit + k_ct * e_lateral
    //
    // Si el robot está a la izquierda del segmento (e > 0) → necesita girar derecha → Δω < 0
    // Si está a la derecha (e < 0) → necesita girar izquierda → Δω > 0
    // Por eso el signo es NEGATIVO: restamos k * error
    cross_track_error_ = computeCrossTrackError(state);
    const double omega_correction = -p_.k_crosstrack * cross_track_error_;

    // ── PASO 8: Velocidad lineal adaptativa ───────────────────────────────────
    // Curvatura alta → frenar. Cerca de la meta → frenar.
    // También aplicamos rampa de aceleración para no dar saltos bruscos.
    const double v_desired = computeVelocity(kappa, state);

    // ── PASO 9: Ensamblar comando final ───────────────────────────────────────
    // ω = curvatura * v  +  corrección_lateral
    //
    // La ω de Pure Pursuit viene de:  ω = κ * v
    // (derivación: si el radio del arco es R = 1/κ, y v = ω * R → ω = v/R = v*κ)
    const double omega = kappa * v_desired + omega_correction;

    return ControlOutput{v_desired, omega};
}


// ════════════════════════════════════════════════════════════════════════════
//  SUBMÉTODOS PRIVADOS
// ════════════════════════════════════════════════════════════════════════════

// ── computeLd ────────────────────────────────────────────────────────────────
// 'const' al final de la firma: este método NO modifica ningún miembro.
// El compilador lo garantiza y puede hacer optimizaciones adicionales.
double PurePursuitController::computeLd(double v) const
{
    // Clamp mínimo: aunque v = 0, siempre miramos al menos ld_min hacia adelante.
    return p_.ld_k * std::abs(v) + p_.ld_min;
}


// ── updateClosestIdx ─────────────────────────────────────────────────────────
// Avanza closest_idx_ mientras el waypoint siguiente esté más cerca que el actual.
// Esto hace que el algoritmo siempre progrese hacia adelante en el path,
// nunca regrese aunque el robot se desvíe lateralmente.
void PurePursuitController::updateClosestIdx(const RobotState& state)
{
    // static_cast<int>: conversión explícita de size_t (sin signo) a int (con signo).
    // Necesaria para comparar con closest_idx_ que es int.
    // Sin el cast, el compilador puede advertir sobre comparar tipos distintos.
    const int n = static_cast<int>(path_.size());

    // Avanzamos mientras el SIGUIENTE waypoint esté más cerca que el actual.
    // Condición: closest_idx_ + 1 < n  evita salirse del vector.
    while (closest_idx_ + 1 < n) {
        const double d_current = std::hypot(
            path_[closest_idx_].x - state.x,
            path_[closest_idx_].y - state.y);

        const double d_next = std::hypot(
            path_[closest_idx_ + 1].x - state.x,
            path_[closest_idx_ + 1].y - state.y);

        if (d_next < d_current) {
            ++closest_idx_;   // ++i es ligeramente más eficiente que i++ para enteros
        } else {
            break;   // El actual ya es el más cercano, paramos
        }
    }
}


// ── findLookahead ─────────────────────────────────────────────────────────────
// Busca la intersección más adelantada del círculo de radio 'ld' centrado
// en el robot con los segmentos del path (desde closest_idx_ en adelante).
//
// Geometría: para cada segmento P1→P2, planteamos:
//   |P1 + t*(P2-P1) - robot|² = ld²
// Eso es una ecuación cuadrática en t. Si t ∈ [0,1], hay intersección.
std::optional<Point2D> PurePursuitController::findLookahead(
    const RobotState& state, double ld) const
{
    const int n = static_cast<int>(path_.size());

    // Guardaremos el mejor candidato encontrado.
    // std::optional empieza vacío — como un puntero que empieza en nullptr,
    // pero sin el peligro de dereferenciarlo accidentalmente.
    std::optional<Point2D> best;
    int   best_seg{-1};
    double best_t{-1.0};

    // Iteramos desde el segmento activo hasta el penúltimo waypoint.
    // El segmento i va de path_[i] a path_[i+1].
    for (int i = closest_idx_; i < n - 1; ++i) {
        const Point2D& p1 = path_[i];
        const Point2D& p2 = path_[i + 1];

        // Vector del segmento: dirección de p1 a p2
        const double dx = p2.x - p1.x;
        const double dy = p2.y - p1.y;

        // Vector del robot a p1
        const double fx = p1.x - state.x;
        const double fy = p1.y - state.y;

        // Coeficientes de la cuadrática  a*t² + b*t + c = 0
        // a = |d|²   (siempre positivo)
        // b = 2*(f·d)
        // c = |f|² - ld²
        const double a   = dx*dx + dy*dy;
        const double b   = 2.0 * (fx*dx + fy*dy);
        const double c   = fx*fx + fy*fy - ld*ld;
        const double disc = b*b - 4.0*a*c;   // discriminante

        // disc < 0 → no hay intersección real con este segmento
        if (disc < 0.0) continue;

        const double sq = std::sqrt(disc);

        // Las dos soluciones de la cuadrática.
        // t2 > t1 siempre. t2 es la intersección más adelantada en el segmento.
        const double t1 = (-b - sq) / (2.0 * a);
        const double t2 = (-b + sq) / (2.0 * a);

        // Revisamos t2 primero (más adelante), luego t1.
        // Un t válido debe estar en [0, 1] (dentro del segmento).
        for (const double t : {t2, t1}) {
            if (t < 0.0 || t > 1.0) continue;   // fuera del segmento

            // Este segmento es más adelante que el mejor hasta ahora
            if (i > best_seg || (i == best_seg && t > best_t)) {
                best_seg = i;
                best_t   = t;
                // El punto de intersección: P1 + t * (P2-P1)
                best = Point2D{p1.x + t*dx, p1.y + t*dy};
            }
        }
    }

    return best;   // Vacío si no encontró nada, con valor si sí.
}


// ── computeCrossTrackError ────────────────────────────────────────────────────
// Distancia perpendicular con signo del robot al segmento activo.
//
// Truco geométrico: el producto cruzado 2D de dos vectores da el área del
// paralelogramo que forman. Dividido por la longitud del segmento da la
// distancia perpendicular.
//
//   e = (P2-P1) × (robot-P1) / |P2-P1|
//
// El "×" aquí es el escalar:  ax*by - ay*bx
// Signo positivo = robot a la izquierda del segmento (mirando hacia adelante).
double PurePursuitController::computeCrossTrackError(const RobotState& state) const
{
    if (closest_idx_ >= static_cast<int>(path_.size()) - 1) {
        return 0.0;   // En el último waypoint no hay segmento activo
    }

    const Point2D& p1 = path_[closest_idx_];
    const Point2D& p2 = path_[closest_idx_ + 1];

    // Vector del segmento
    const double seg_x = p2.x - p1.x;
    const double seg_y = p2.y - p1.y;
    const double seg_len = std::hypot(seg_x, seg_y);

    if (seg_len < 1e-9) return 0.0;   // Segmento degenerado (dos waypoints iguales)

    // Vector del inicio del segmento al robot
    const double to_robot_x = state.x - p1.x;
    const double to_robot_y = state.y - p1.y;

    // Producto cruzado 2D (el escalar z del cross product 3D)
    // Divide por longitud del segmento para obtener distancia perpendicular
    return (seg_x * to_robot_y - seg_y * to_robot_x) / seg_len;
}


// ── computeVelocity ───────────────────────────────────────────────────────────
// Calcula la velocidad lineal deseada considerando:
//   1. Curvatura actual (frenar en curvas)
//   2. Distancia al final (frenar antes de la meta)
//   3. Rampa de aceleración (no saltar bruscamente de v)
double PurePursuitController::computeVelocity(double kappa, const RobotState& state)
{
    // 1) Velocidad base según curvatura
    //    v = v_max / (1 + k_c * |κ|)
    //    Curvatura cero → v_max. Curvatura alta → v baja.
    const double v_curvature = p_.v_max / (1.0 + p_.k_curvature * std::abs(kappa));

    // 2) Velocidad de frenado por proximidad a la meta
    //    Rampa lineal: va de v_max a 0 en el tramo final stop_dist.
    const Point2D& goal = path_.back();
    const double dist = std::hypot(goal.x - state.x, goal.y - state.y);
    const double v_stop = (dist < p_.stop_dist)
        ? p_.v_max * (dist / p_.stop_dist)   // fracción de v_max según distancia restante
        : p_.v_max;

    // 3) El mínimo de los dos límites anteriores
    //    std::min devuelve el menor de dos valores del mismo tipo.
    double v_desired = std::min(v_curvature, v_stop);

    // 4) Rampa de aceleración: limita cuánto puede cambiar v en un tick.
    //    Sin esto, si v_desired salta de 0 a v_max instantáneamente,
    //    el robot podría dar un tirón mecánico o el PID de rueda no alcanza.
    //
    //    Necesitamos dt para la rampa. Como la clase no conoce el tiempo,
    //    usamos una aproximación: asumimos dt ≈ 0.05s (20Hz).
    //    En una implementación más precisa, pasarías dt como parámetro a compute().
    constexpr double dt = 0.05;   // 'constexpr': el compilador evalúa esto en tiempo
                                   // de compilación, no en runtime. Es una constante
                                   // real, no una variable.
    const double dv_max = p_.a_max * dt;

    // Clamp: limita v_desired al rango [v_prev_ - dv_max, v_prev_ + dv_max]
    // std::clamp(valor, min, max) — disponible desde C++17
    v_desired = std::clamp(v_desired, v_prev_ - dv_max, v_prev_ + dv_max);

    // Guardamos para el siguiente tick
    v_prev_ = v_desired;

    return v_desired;
}


// ════════════════════════════════════════════════════════════════════════════
//  GETTERS — const porque no modifican nada
// ════════════════════════════════════════════════════════════════════════════

bool PurePursuitController::goalReached() const
{
    return goal_reached_;
}

Point2D PurePursuitController::getLookaheadPoint() const
{
    return lookahead_point_;
}

double PurePursuitController::getCrossTrackError() const
{
    return cross_track_error_;
}

double PurePursuitController::getCurrentLd() const
{
    return current_ld_;
}