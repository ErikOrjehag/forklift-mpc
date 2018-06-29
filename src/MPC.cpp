

#ifndef MPC_PRIVATE
#define MPC_PRIVATE
#include "MPC.h"
#undef MPC_PRIVATE
#endif // MPC_PRIVATE


class FG_eval
{
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    const Eigen::VectorXd& K;
    float desiredSpeed;

    FG_eval(const Eigen::VectorXd& K, float desiredSpeed)
        : K(K), desiredSpeed(desiredSpeed)
    {
    }

    void operator()(ADvector& fg, const ADvector& x)
    {
        // Cost in position 0, rest is constraints
        fg[0] = 0.0;

        // Cost of current state error
        for (int i = 0; i < N; i++) {
            const auto cte = x[ID_FIRST_cte + i];
            const auto eheading = x[ID_FIRST_eheading + i];
            const auto v = x[ID_FIRST_v + i] - desiredSpeed;
            fg[0] += (W_cte*cte*cte + W_eheading*eheading*eheading + W_v*v*v);
        }

        // Cost of using actuations to correct the error
        for (int i = 0; i < N - 1; i++) {
            const auto steer_ac = x[ID_FIRST_steer_ac + i];
            const auto v_ac = x[ID_FIRST_v_ac + i];
            fg[0] += (W_steer_ac*steer_ac*steer_ac + W_v_ac*v_ac*v_ac);
        }

        // Cost of changing the actuations quickly
        for (int i = 0; i < N - 2; i++) {
            const auto dsteer_ac = x[ID_FIRST_steer_ac + i + 1] - x[ID_FIRST_steer_ac + i];
            const auto dv_ac = x[ID_FIRST_v_ac + i + 1] - x[ID_FIRST_v_ac + i];
            fg[0] += (W_dsteer_ac*dsteer_ac*dsteer_ac + W_dv_ac*dv_ac*dv_ac);
      }

      // Constraints given the state does not change
      fg[1 + ID_FIRST_px]       = x[ID_FIRST_px];
      fg[1 + ID_FIRST_py]       = x[ID_FIRST_py];
      fg[1 + ID_FIRST_heading]  = x[ID_FIRST_heading];
      fg[1 + ID_FIRST_steer]    = x[ID_FIRST_steer];
      fg[1 + ID_FIRST_v]        = x[ID_FIRST_v];
      fg[1 + ID_FIRST_cte]      = x[ID_FIRST_cte];
      fg[1 + ID_FIRST_eheading] = x[ID_FIRST_eheading];

       // Constraints based on our kinematic model
      for (int i = 0; i < N - 1; i++) {

        // For readability
        const int ID_CURRENT_px        = i + ID_FIRST_px;
        const int ID_CURRENT_py        = i + ID_FIRST_py;
        const int ID_CURRENT_heading   = i + ID_FIRST_heading;
        const int ID_CURRENT_steer     = i + ID_FIRST_steer;
        const int ID_CURRENT_v         = i + ID_FIRST_v;
        const int ID_CURRENT_cte       = i + ID_FIRST_cte;
        const int ID_CURRENT_eheading  = i + ID_FIRST_eheading;
        const int ID_CURRENT_v_ac      = i + ID_FIRST_v_ac;
        const int ID_CURRENT_steer_ac  = i + ID_FIRST_steer_ac;

        // Current state and actuations
        const auto px0       = x[ID_CURRENT_px];
        const auto py0       = x[ID_CURRENT_py];
        const auto heading0  = x[ID_CURRENT_heading];
        const auto steer0    = x[ID_CURRENT_steer];
        const auto v0        = x[ID_CURRENT_v];
        const auto cte0      = x[ID_CURRENT_cte];
        const auto eheading0 = x[ID_CURRENT_heading];
        const auto v_ac0     = x[ID_CURRENT_v_ac];
        const auto steer_ac0 = x[ID_CURRENT_steer_ac];

        // Next state
        const auto px1       = x[1 + ID_CURRENT_px];
        const auto py1       = x[1 + ID_CURRENT_py];
        const auto heading1  = x[1 + ID_CURRENT_heading];
        const auto steer1    = x[1 + ID_CURRENT_steer];
        const auto v1        = x[1 + ID_CURRENT_v];
        const auto cte1      = x[1 + ID_CURRENT_cte];
        const auto eheading1 = x[1 + ID_CURRENT_eheading];

        // Desired py and heading
        const auto py_desired = K[3] * px0 * px0 * px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
        const auto heading_desired = CppAD::atan(3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]);

        // Relationship of current state + actuations and next state

        ForkliftModel<CppAD::AD<double>> model;
        model.position[0] = px0;
        model.position[1] = py0;
        model.heading = heading0;
        model.steer = steer0;
        model.speed = v0;
        model.update(DT);

        /*const auto cte1_f = py_desired - model.position[0]; // maybe invert?
        const auto eheading1_f = model.heading - heading_desired; // ?
        const auto v1_f = v0;
        const auto steer1_f = steer0;
        const auto px1_f = model.position[0];
        const auto py1_f = model.position[1];
        const auto heading1_f = model.heading;*/

        const auto px1_f = px0 + model.dx;
        const auto py1_f = py0 + model.dy;
        const auto heading1_f = heading0 + model.da;
        const auto v1_f = v0 - v0 + v_ac0;
        const auto steer1_f = steer0 - steer0 + steer_ac0;
        const auto cte1_f = py_desired - py0 + model.dy;
        const auto eheading1_f = heading0 - heading_desired + model.da;

        // Store the constraint expression of two consecutive states
        fg[2 + ID_CURRENT_px]      = px1 - px1_f;
        fg[2 + ID_CURRENT_py]      = py1 - py1_f;
        fg[2 + ID_CURRENT_heading] = heading1 - heading1_f;
        fg[2 + ID_CURRENT_steer]   = steer1 - steer1_f;
        fg[2 + ID_CURRENT_v]       = v1 - v1_f;
        fg[2 + ID_FIRST_cte]       = cte1 - cte1_f;
        fg[2 + ID_FIRST_eheading]  = eheading1 - eheading1_f;
      }
    }
};

MPC::MPC(ForkliftModel<double>& model)
    : model(model)
{
    x.resize(NX);
    xLowerBound.resize(NX);
    xUpperBound.resize(NX);
    gLowerBound.resize(NG);
    gUpperBound.resize(NG);

    // Set all states to zero
    for (int i = 0; i < x.size(); i++) {
        x[i] = 0;
    }

    // Limits of state variables
    for (int i = 0; i < ID_FIRST_steer; i++) {
        xLowerBound[i] = -1.0e10;
        xUpperBound[i] = 1.0e10;
    }

    for (int i = ID_FIRST_steer; i < ID_FIRST_v; i++) {
        xLowerBound[i] = -model.MAX_STEER;
        xUpperBound[i] = model.MAX_STEER;
    }

    for (int i = ID_FIRST_v; i < ID_FIRST_steer_ac; i++) {
        xLowerBound[i] = -model.MAX_SPEED;
        xUpperBound[i] = model.MAX_SPEED;
    }

    // Limits of actuation inputs
    for (int i = ID_FIRST_steer_ac; i < ID_FIRST_v_ac; i++) {
        xLowerBound[i] = -model.MAX_STEER;
        xUpperBound[i] = model.MAX_STEER;
    }

    for (int i = ID_FIRST_v_ac; i < NX; i++) {
        xLowerBound[i] = -model.MAX_SPEED;
        xUpperBound[i] = model.MAX_SPEED;
    }

    /*  The first constraint for each state variable
        refers to the initial state conditions. This
        will be initialized when solve() is called.
        The succeeding constraints refer to the
        relationship between succeeding states based
        on our kinematic model of the system. */
    for (int i = 0; i < NG; i++) {
        gLowerBound[i] = 0.0;
        gUpperBound[i] = 0.0;
    }
}

void MPC::solve(const Eigen::VectorXd& K, double desiredSpeed)
{
    // In local frame of reference
    const double px = 0;
    const double py = 0;
    const double heading = 0;
    const double steer = model.steer;
    const double v = model.speed;
    const double cte = K[0];
    const double eheading = -atan(K[1]);

    x[ID_FIRST_px] = px;
    x[ID_FIRST_py] = py;
    x[ID_FIRST_heading] = heading;
    x[ID_FIRST_steer] = steer;
    x[ID_FIRST_v] = v;
    x[ID_FIRST_cte] = cte;
    x[ID_FIRST_eheading] = eheading;

    gLowerBound[ID_FIRST_px] = px;
    gLowerBound[ID_FIRST_py] = py;
    gLowerBound[ID_FIRST_heading] = heading;
    gLowerBound[ID_FIRST_steer] = steer;
    gLowerBound[ID_FIRST_v] = v;
    gLowerBound[ID_FIRST_cte] = cte;
    gLowerBound[ID_FIRST_eheading] = eheading;

    gUpperBound[ID_FIRST_px] = px;
    gUpperBound[ID_FIRST_py] = py;
    gUpperBound[ID_FIRST_heading] = heading;
    gUpperBound[ID_FIRST_steer] = steer;
    gUpperBound[ID_FIRST_v] = v;
    gUpperBound[ID_FIRST_cte] = cte;
    gUpperBound[ID_FIRST_eheading] = eheading;


    //**************************************************************
    //* SOLVE
    //**************************************************************

    // object that computes objective and constraints
    FG_eval fgEval(K, desiredSpeed);

    // options for IPOPT solver
    std::string options;

    //options += "Retape  true\n";

    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options,
        x,
        xLowerBound,
        xUpperBound,
        gLowerBound,
        gUpperBound,
        fgEval,
        solution);

    // comment out the lines below to debug!
    bool ok = true;
    auto cost = solution.obj_value;

    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    if (ok) {
        //std::cout << "OK! Cost:" << cost << std::endl;
    } else {
        std::cout << "SOMETHING IS WRONG!" << cost << std::endl;
    }

    //**************************************************************
    //* STORE RELEVANT INFORMATION FROM SOLUTION
    //**************************************************************

    this->steer_ac = solution.x[ID_FIRST_steer_ac];
    this->speed_ac = solution.x[ID_FIRST_v_ac];

    //std::cout << "result (steer_ac: " << this->steer_ac << ", speed_ac: " << this->speed_ac << ")" << std::endl;

    this->prediction.clear();

    for (int i = 0; i < N; ++i) {
        const double px = solution.x[ID_FIRST_px + i];
        const double py = solution.x[ID_FIRST_py + i];
        this->prediction.push_back(Eigen::Vector2d(px, py));
    }

}


void MPC::draw(sf::RenderWindow& window, TransformStack& ts)
{
    std::vector<sf::Vertex> vertices;
    for (Eigen::Vector2d& p : prediction) vertices.push_back(sf::Vertex(sf::Vector2f(p[0], p[1]), sf::Color::Blue));
    window.draw(&vertices[0], vertices.size(), sf::LineStrip, ts);
}
