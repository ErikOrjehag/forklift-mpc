#include "MPC.h"

#ifndef MPC_PRIVATE
#define MPC_PRIVATE
#include "NavError.h"
#undef MPC_PRIVATE
#endif // MPC_PRIVATE


class FG_eval
{
public:
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
    const Eigen::VectorXd& K;

    FG_eval(const Eigen::VectorXd& K)
        : K(K)
    {
    }

    void operator()(ADvector& fg, const ADvector& x)
    {
        // Cost

        fg[0] = 0.0;

        for (int i = 0; i < mpc.N; i++) {
            const auto cte = x[mpc.ID_FIRST_cte + i];
            const auto epsi = x[mpc.ID_FIRST_epsi + i];
            const auto v = x[mpc.ID_FIRST_v + i] - 1.5;
            fg[0] += (mpc.W_cte * cte * cte + mpc.W_epsi * epsi * epsi + mpc.W_v * v * v);
        }

        for (int i = 0; i < mpc.N - 1; ++i) {
            const auto steer = x[mpc.ID_FIRST_steer + i];
            const auto a = x[mpc.ID_FIRST_a + i];
            fg[0] += (mpc.W_steer * steer * steer + mpc.W_a * a * a);
        }

        for (int i = 0; i < mpc.N - 2; ++i) {
            const auto dsteer = x[mpc.ID_FIRST_steer + i + 1] - x[mpc.ID_FIRST_steer + i];
            const auto da = x[mpc.ID_FIRST_a + i + 1] - x[mpc.ID_FIRST_a + i];
            fg[0] += (mpc.W_dsteer * dsteer * dsteer + mpc.W_da * da * da);
      }

      // Constraints

      fg[mpc.ID_FIRST_px + 1] = x[mpc.ID_FIRST_px];
      fg[mpc.ID_FIRST_py + 1] = x[mpc.ID_FIRST_py];
      fg[mpc.ID_FIRST_psi + 1] = x[mpc.ID_FIRST_psi];
      fg[mpc.ID_FIRST_v + 1] = x[mpc.ID_FIRST_v];
      fg[mpc.ID_FIRST_cte + 1] = x[mpc.ID_FIRST_cte];
      fg[mpc.ID_FIRST_epsi + 1] = x[mpc.ID_FIRST_epsi];

       // constraints based on our kinematic model
      for (int i = 0; i < mpc.N - 1; ++i) {
            // where the current state variables of interest are stored
            // stored for readability
            const int ID_CURRENT_px = mpc.ID_FIRST_px + i;
            const int ID_CURRENT_py = mpc.ID_FIRST_py + i;
            const int ID_CURRENT_psi = mpc.ID_FIRST_psi + i;
            const int ID_CURRENT_v = mpc.ID_FIRST_v + i;
            const int ID_CURRENT_cte = mpc.ID_FIRST_cte + i;
            const int ID_CURRENT_epsi = mpc.ID_FIRST_epsi + i;
            const int ID_CURRENT_steer = mpc.ID_FIRST_steer + i;
            const int ID_CURRENT_a = mpc.ID_FIRST_a + i;

            //current state and actuations
            const auto px0 = x[ID_CURRENT_px];
            const auto py0 = x[ID_CURRENT_py];
            const auto psi0 = x[ID_CURRENT_psi];
            const auto v0 = x[ID_CURRENT_v];
            const auto cte0 = x[ID_CURRENT_cte];
            const auto epsi0 = x[ID_CURRENT_epsi];
            const auto delta0 = x[ID_CURRENT_steer];
            const auto a0 = x[ID_CURRENT_a];

            // next state
            const auto px1 = x[ID_CURRENT_px + 1];
            const auto py1 = x[ID_CURRENT_py + 1];
            const auto psi1 = x[ID_CURRENT_psi + 1];
            const auto v1 = x[ID_CURRENT_v + 1];
            const auto cte1 = x[ID_CURRENT_cte + 1];
            const auto epsi1 = x[ID_CURRENT_epsi + 1];

            /*// relationship of current state + actuations and next state
            // based on our kinematic model
            Forklift fork = forklift;
            fork.frame.pos[0] = px0;
            fork.frame.pos[1] = py0;
            fork.frame.angle = psi0;
            fork.speed = v0 + a0 * mpc.DT;
            fork.frame = fork.predict(mpc.DT);
            NavError navErr = NavError.calcNavError(path, fork.frame);

            const auto px1_f = fork.frame.pos[0];
            const auto py1_f = fork.frame.pos[1];
            const auto psi1_f = fork.frame.angle;
            const auto v1_f = fork.speed;
            const auto cte1_f = navErr.linear;
            const auto epsi1_f = navErr.angle;

            // store the constraint expression of two consecutive states
            fg[ID_CURRENT_px + 2] = px1 - px1_f;
            fg[ID_CURRENT_py + 2] = py1 - py1_f;
            fg[ID_CURRENT_psi + 2] = psi1 - psi1_f;
            fg[ID_CURRENT_v + 2] = v1 - v1_f;
            fg[ID_CURRENT_cte + 2] = cte1 - cte1_f;
            fg[ID_CURRENT_epsi + 2] = epsi1 - epsi1_f;*/
      }
    }
};

MPC::MPC(ForkliftModel<double>& model)
    : model(model)
{
    x.resize(NX);

    for (int i = 0; i < x.size(); i++) {
        x[i] = 0;
    }

    xLowerBound.resize(NX);
    xUpperBound.resize(NX);

    for (int i = 0; i < ID_FIRST_steer_ac; i++) {
        xLowerBound[i] = -1.0e10;
        xUpperBound[i] = 1.0e10;
    }

    for (int i = ID_FIRST_steer_ac; i < ID_FIRST_v_ac; i++) {
        xLowerBound[i] = M_PI / -2;
        xUpperBound[i] = M_PI / 2;
    }

    for (int i = ID_FIRST_v_ac; i < NX; i++) {
        xLowerBound[i] = -model.MAX_SPEED;
        xUpperBound[i] = model.MAX_SPEED;
    }

    gLowerBound.resize(NG);
    gUpperBound.resize(NG);

    for (int i = 0; i < NG; i++) {
        gLowerBound[i] = 0.0;
        gUpperBound[i] = 0.0;
    }
}

MPC::~MPC()
{
    //dtor
}

void MPC::solve(const Eigen::VectorXd& K)
{
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
    FG_eval fgEval(K);

    // options for IPOPT solver
    std::string options;
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
        std::cout << "OK! Cost:" << cost << std::endl;
    } else {
        std::cout << "SOMETHING IS WRONG!" << cost << std::endl;
    }

    //**************************************************************
    //* STORE RELEVANT INFORMATION FROM SOLUTION
    //**************************************************************

    this->steer_ac = solution.x[ID_FIRST_steer_ac];
    this->v_ac = solution.x[ID_FIRST_v_ac];


    this->prediction.clear();

    for (int i = 0; i < N; ++i) {
        const double px = solution.x[ID_FIRST_px + i];
        const double py = solution.x[ID_FIRST_py + i];
        this->prediction.emplace_back(Eigen::Vector2d(px, py));
    }

}


void MPC::draw(sf::RenderWindow& window, TransformStack& ts)
{
    std::vector<sf::Vertex> vertices;
    for (Eigen::Vector2d& p : prediction) vertices.push_back(sf::Vertex(sf::Vector2f(p[0], p[1]), sf::Color::Yellow));
    window.draw(&vertices[0], vertices.size(), sf::LineStrip, ts);
}
