#pragma once          

#include <vector>    
#include <cmath>       
#include <limits>      
#include <stdexcept>   
#include <optional>   

struct Point2D {
    double x{0.0}; //initilize as 0. using {} to avoid allocating garbage
    double y{0.0};   
};


struct RobotState { //struct to parse odometry msg directly 
    double x{0.0};     
    double y{0.0};      
    double theta{0.0};  // yaw
    double v{0.0};      // lineal vel
};

struct ControlOutput { //twist message directly
    double v{0.0};      // lineal velocity
    double omega{0.0};  // angular velocity
};


struct Params { //struct for all controller params in one
    
    double ld_min{0.3};        // Lookahead min (front target in m)
    double ld_k{0.5};          // adaptative gain for twisting Ld = ld_k * v + ld_min
                               //more speed, lookup farther which is more controlled

    double v_max{0.3};         // max lineal velocity
    double k_curvature{2.0};   // Frenado por curvatura: v = v_max / (1 + k_c * |κ|)
                               // Cuando la trayectoria curva, el robot frena solo.
    double a_max{0.5};         // Aceleración máxima [m/s²]. Limita saltos bruscos de v.

    double k_crosstrack{0.5};  // gain P for lateral error
                            

    double wheelbase{0.19};    // distance between wheels

    double goal_tol{0.02};  //tolerance to goal 
    double stop_dist{0.5};     //once reached, the robot starts stopping
};


class PurePursuitController {
public:

    explicit PurePursuitController(const Params& p); //class starts with params (struct)

    void setPath(const std::vector<Point2D>& path);

    ControlOutput compute(const RobotState& state); //returns struct with robot state

    void reset(); //call reset after every new path recieved

    bool    goalReached()        const;  
    Point2D getLookaheadPoint()  const;  
    double  getCrossTrackError() const;  
    double  getCurrentLd()       const;  //lookahead actual distance

private:

    Params p_;   

    std::vector<Point2D> path_; //vector for storing path
    int    closest_idx_{0};    //index of current waypoint in vector path
    bool   goal_reached_{false}; //flag to verify goal has been reached

    Point2D lookahead_point_{};
    double  cross_track_error_{0.0};
    double  current_ld_{0.0};
    double  v_prev_{0.0};      // Velocidad del tick anterior (para rampa de aceleración)


    //private methods
    double computeLd(double v) const; // comiputes Lookaheadd = ld_k * v + ld_min


    void updateClosestIdx(const RobotState& state); //updates index in path


    std::optional<Point2D> findLookahead(const RobotState& state, double ld) const; //intersection with circle for LD
                                                                                    //returns intersec if true (xy)

    // Distancia perpendicular con signo del robot al segmento activo.
    // Positivo = robot está a la izquierda del segmento (mirando hacia adelante).
    // Negativo = robot está a la derecha.
    double computeCrossTrackError(const RobotState& state) const;

    double computeVelocity(double kappa, const RobotState& state); //computes lineal vel according to curvature and distance to goal
};