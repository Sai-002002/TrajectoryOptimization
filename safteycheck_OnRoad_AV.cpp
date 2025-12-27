
// The "Dynamic Gap Acceptance" Check (High Probability)
//The Context: Before generating a trajectory, 
//the planner must decide if a gap in the target lane is safe. 
//Since cars are moving, checking the current position is not enough; 
//you must predict future positions.
//
struct Vehicle { double s; double v; }; // s = longitudinal position // condidering frenet coordinates

bool isLaneChangeSafe(Vehicle ego, vector<Vehicle> target_lane_cars, double time_horizon) {
    double min_dist = 10.0;

    for (auto obs : target_lane_cars) {
        // Check 1: Current Snapshot
        if (abs(obs.s - ego.s) < min_dist) return false;

        // Check 2: Future Prediction (s = s0 + v*t)
        double ego_future = ego.s + ego.v * time_horizon;
        double obs_future = obs.s + obs.v * time_horizon;
        
        if (abs(obs_future - ego_future) < min_dist) return false;
    }
    return true;
}
//2. The "Cost Function" Evaluator (Medium Probability)
//The Context: 
//The planner often generates multiple candidate trajectories
// (e.g., one fast lane change, one slow one, one abort). 
//You need to pick the best one.
double calculateCost(const Trajectory& traj, Point obstacle) {
    double total_cost = 0;
    double w_jerk = 10.0;
    double w_dist = 100.0;

    for (size_t i = 1; i < traj.points.size(); i++) {
        // 1. Calc Acceleration (dv/dt)
        double dv = traj[i].v - traj[i-1].v;
        double dt = traj[i].t - traj[i-1].t;
        double accel = dv / dt;
        
        // Penalize high accel (Jerky)
        total_cost += w_jerk * (accel * accel);

        // 2. Calc Distance to Obstacle
        double dist = distance(traj[i], obstacle);
        if (dist < 1.0) return INFINITY; // Collision
        total_cost += w_dist * (1.0 / dist); // Closer = Higher Cost
    }
    return total_cost;
}
//3. The "Frenet to Cartesian" Logic (Math Focus)
//The Context: AV  rely heavily on Frenet Frames ($s$ = distance along lane, $d$ = lateral offset) for lane changing.

//The Prompt:

//"Assuming a straight road (reference line is the X-axis), convert a target lane change trajectory from Frenet coordinates to Cartesian coordinates.

//The trajectory is defined as a polynomial for lateral movement: $d(t) = a_0 + a_1t + a_2t^2...$
//The longitudinal velocity is constant $v_x$.
//Output the $(x, y)$ coordinates for the next 5 seconds."
//Key Challenges:

//Understanding that for a straight road, $x(t) \approx s(t)$ and $y(t) = d(t)$.
//Evaluating the polynomial inside a loop.
//Sample Solution Logic:

// d(t) = a0 + a1*t + a2*t^2 + a3*t^3 (Cubic spline for lane change)
struct Coeffs { double a0, a1, a2, a3; };

vector<Point> generatePath(Coeffs c, double vx, double T) {
    vector<Point> path;
    double dt = 0.1;
    
    for(double t = 0; t <= T; t += dt) {
        double x = vx * t; // Longitudinal movement
        
        // Lateral movement (evaluating the polynomial)
        double y = c.a0 + c.a1*t + c.a2*t*t + c.a3*t*t*t;
        
        path.push_back({x, y});
    }
    return path;
}