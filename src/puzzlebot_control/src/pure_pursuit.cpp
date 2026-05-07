//this is not a ros2 node, is pure pursuit raw algorithm

#include "puzzlebot_control/pure_pursuit_controller.hpp"
#include <algorithm>

PurePursuitController::PurePursuitController(const Params& p)
    : p_(p)                    // copia los parámetros al miembro p_
    , closest_idx_(0)          // start from first watpoint (segmento)
    , goal_reached_(false)
    , cross_track_error_(0.0)
    , current_ld_(p.ld_min)
    , v_prev_(0.0)
{
}


void PurePursuitController::setPath(const std::vector<Point2D>& path){

    if (path.size() < 2) {
        throw std::invalid_argument(
            "[PurePursuit] Path needs at least 2 segments.");
    }
o y
    path_ = path;  //if len > 2, copy the objetct for the class
    reset();        // reset everythinh
}


void PurePursuitController::reset()
{
    closest_idx_      = 0;
    goal_reached_     = false;
    cross_track_error_= 0.0;
    current_ld_       = p_.ld_min;
    v_prev_           = 0.0;
    lookahead_point_  = Point2D{};  //{} sets xy as 0
}


ControlOutput PurePursuitController::compute(const RobotState& state){

    if (path_.empty() || goal_reached_) { //if path empty or goal reached stop
        return ControlOutput{0.0, 0.0};
    }

    const Point2D& goal = path_.back();   // save last waypoint
    const double dist_to_goal = std::hypot(goal.x - state.x, goal.y - state.y); //distance to goal (hypot)

    if (dist_to_goal < p_.goal_tol) { //if distance to goal close enough as goal tolerance, stop
        goal_reached_ = true;
        return ControlOutput{0.0, 0.0};
    }

    updateClosestIdx(state); //move to next index (waypoint)

    // Ld = ld_k * v_actual + ld_min
    current_ld_ = computeLd(state.v); //ld adaptative

    //find el lookahead point
    std::optional<Point2D> lp = findLookahead(state, current_ld_); // intersección del círculo de radio Ld con los segmentos del path.

    // Si no hay intersección (robot muy desviado, path terminando), usamos
    // el último waypoint como fallback para no quedarnos sin comando.
    if (!lp.has_value()) {
        lp = goal;
    }

    lookahead_point_ = lp.value(); //save lookahead for debug

    //transform lookahead point to robot frame
    // dx_g, dy_g: vector global del robot al lookahead point
    const double dx_g = lookahead_point_.x - state.x;
    const double dy_g = lookahead_point_.y - state.y;

    // rotacion inversa por -theta (local robot frame):
    //   x_local =  cos(θ)*dx + sin(θ)*dy   - forward
    //   y_local = -sin(θ)*dx + cos(θ)*dy   - left
    //

    //only y_local is needed for algoriith
    // const double x_local =  std::cos(state.theta)*dx_g + std::sin(state.theta)*dy_g;
    const double y_local = -std::sin(state.theta)*dx_g + std::cos(state.theta)*dy_g;

    //compute k curvature for knowing how much to twist kappa
    //   κ = 2 * y_local / Ld²

    //   y_local = 0 means the look ahead point is in front (go straight)
    //   y_local > 0  turn left
    //   y_local < 0  turn right
    const double kappa = (2.0 * y_local) / (current_ld_ * current_ld_);

    //   ω_total = ω_pure_pursuit + k_ct * e_lateral

    cross_track_error_ = computeCrossTrackError(state);
    const double omega_correction = -p_.k_crosstrack * cross_track_error_;

    const double v_desired = computeVelocity(kappa, state); //lineal vel adaptative depending on curve and goal distance

    // w = curvature * v  +  lateral correction
    // La ω de Pure Pursuit viene de:  ω = κ * v
    const double omega = kappa * v_desired + omega_correction;

    return ControlOutput{v_desired, omega};
}



double PurePursuitController::computeLd(double v) const //computeLd 
{
    return p_.ld_k * std::abs(v) + p_.ld_min; //even if lineal vel is 0, lookahead minimum
}


// Avanza closest_idx_ mientras el waypoint siguiente este mas cerca que el actual.
// Esto hace que el algoritmo siempre progrese hacia adelante en el path,
// nunca regrese aunque el robot se desvee lateralmente.
void PurePursuitController::updateClosestIdx(const RobotState& state)
{
    const int n = static_cast<int>(path_.size()); //get size of path

    while (closest_idx_ + 1 < n) {
        const double d_current = std::hypot( //compute distance between current state and next waypopint
            path_[closest_idx_].x - state.x, //closest indx is path elements iteration
            path_[closest_idx_].y - state.y);

        const double d_next = std::hypot( //distance between current state and next waypoint 
            path_[closest_idx_ + 1].x - state.x,
            path_[closest_idx_ + 1].y - state.y);

        if (d_next < d_current) { //if distance for next wp, the robot goes forward (avoid going backwards)
            ++closest_idx_;   
        } else {
            break;   // else, stop 
        }
    }
}


//find lookahead
// Busca la interseccion mas adelantada del ctrculo de radio 'ld' centrado
// en el robot con los segmentos del path (desde closest_idx_ en adelante).
// Geometría: para cada segmento P1 to P2, planteamos:
//   |P1 + t*(P2-P1) - robot|² = ld²
// Eso es una ecuacion cuadratica en t. Si t ∈ [0,1], hay interseccion.
std::optional<Point2D> PurePursuitController::findLookahead(
    const RobotState& state, double ld) const
{
    const int n = static_cast<int>(path_.size());

    std::optional<Point2D> best;
    int   best_seg{-1};
    double best_t{-1.0};

    // Iiterate from segmento activo hasta el penultimo waypoint.
    for (int i = closest_idx_; i < n - 1; ++i) {
        const Point2D& p1 = path_[i];
        const Point2D& p2 = path_[i + 1];

        //vector from actual point to next point
        const double dx = p2.x - p1.x;
        const double dy = p2.y - p1.y;

        //vector from robot to point p1
        const double fx = p1.x - state.x;
        const double fy = p1.y - state.y;

        // Coeficientes of quadratic ecuation a*t² + b*t + c = 0
        const double a   = dx*dx + dy*dy;
        const double b   = 2.0 * (fx*dx + fy*dy);
        const double c   = fx*fx + fy*fy - ld*ld;
        const double disc = b*b - 4.0*a*c;   // discriminant

        // disc < 0 means there is no intersection
        if (disc < 0.0) continue;

        const double sq = std::sqrt(disc);

        // Las dos soluciones de la cuadratica
        // t2 > t1 siempre. t2 es la interseccion mas adelantada en el segmento
        const double t1 = (-b - sq) / (2.0 * a);
        const double t2 = (-b + sq) / (2.0 * a);

        // Revisamos t2 primero (mas adelante), luego t1.
        // Un t valido debe estar en [0, 1] (dentro del segmento).
        for (const double t : {t2, t1}) {
            if (t < 0.0 || t > 1.0) continue;   // fuera del segmento

            // Este segmento es mas adelante que el mejor hasta ahora
            if (i > best_seg || (i == best_seg && t > best_t)) {
                best_seg = i;
                best_t   = t;
                // El punto de interseccion: P1 + t * (P2-P1)
                best = Point2D{p1.x + t*dx, p1.y + t*dy};
            }
        }
    }

    return best;   //empty if no best found
}


// Distancia perpendicular con signo del robot al segmento activo
//
//el producto cruzado 2D de dos vectores da el área del
// paralelogramo que forman. Dividido por la longitud del segmento da la
// distancia perpendicular
//
//   e = (P2-P1) × (robot-P1) / |P2-P1|
//
// Signo positivo = robot a la izquierda del segmento (mirando hacia adelante).
double PurePursuitController::computeCrossTrackError(const RobotState& state) const
{
    if (closest_idx_ >= static_cast<int>(path_.size()) - 1) {
        return 0.0;   //in last waypoint no aplica
    }

    const Point2D& p1 = path_[closest_idx_]; //get coordinate of current and next point
    const Point2D& p2 = path_[closest_idx_ + 1];

    const double seg_x = p2.x - p1.x; //vector del segmento
    const double seg_y = p2.y - p1.y;
    const double seg_len = std::hypot(seg_x, seg_y);

    if (seg_len < 1e-9) return 0.0;   //segmento degenerado (dos waypoints iguales)

    const double to_robot_x = state.x - p1.x; //vector del inicio del segmento al robot
    const double to_robot_y = state.y - p1.y;

    // Producto cruzado 2D (el escalar z del cross product 3D)
    // Divide por longitud del segmento para obtener distancia perpendicular
    return (seg_x * to_robot_y - seg_y * to_robot_x) / seg_len;
}


//get lineal velocity cosnidering:
//   1. kappa (curvature)
//   2. distance to goal
//   3. Rampa de aceleracion
double PurePursuitController::computeVelocity(double kappa, const RobotState& state)
{
    //    v = v_max / (1 + k_c * |κ|)
    //    Curvatura is 0 so v_max
    //hight curvature, low vel
    const double v_curvature = p_.v_max / (1.0 + p_.k_curvature * std::abs(kappa));

    //Velocidad de frenado por proximidad a la meta
    //    Rampa lineal: va de v_max a 0 en el tramo final stop_dist.
    const Point2D& goal = path_.back();
    const double dist = std::hypot(goal.x - state.x, goal.y - state.y);
    const double v_stop = (dist < p_.stop_dist)
        ? p_.v_max * (dist / p_.stop_dist)   // fracción de v_max según distancia restante
        : p_.v_max;

    // 3) El minimo de los dos límites anteriores
    //    std::min devuelve el menor de dos valores del mismo tipo.
    double v_desired = std::min(v_curvature, v_stop);

    // 4) Rampa de aceleracion: limita cuánto puede cambiar v en un tick.
    //    Sin esto, si v_desired salta de 0 a v_max instantaneamente,
    //   avoid tirones or lineal vel saturation

    //consider giving dt to compute
    constexpr double dt = 0.05;   
    const double dv_max = p_.a_max * dt; //max accleleration increases progressively

    //limitf lineal vel 
    // std::clamp(valor, min, max) — disponible desde C++17
    v_desired = std::clamp(v_desired, v_prev_ - dv_max, v_prev_ + dv_max);

    v_prev_ = v_desired;

    return v_desired;
}



//getters
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