#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Forklift.h"

#ifdef MPC_PRIVATE
const double DT = 0.016;
const int N = 20; // How many states we "lookahead" in the future

const int NUMBER_OF_STATES = 7; // px, py, heading, steer, v, cte, eheading
const int NUMBER_OF_ACTUATIONS = 2; // steer, v

const int NX = N * NUMBER_OF_STATES + (N - 1) * NUMBER_OF_ACTUATIONS; // Number of states + actuations
const int NG = N * NUMBER_OF_STATES; // Number of constraints

const int ID_FIRST_px = 0;
const int ID_FIRST_py = ID_FIRST_px + N;
const int ID_FIRST_heading = ID_FIRST_py + N;
const int ID_FIRST_cte = ID_FIRST_heading + N;
const int ID_FIRST_eheading = ID_FIRST_cte + N;
const int ID_FIRST_steer = ID_FIRST_eheading + N;
const int ID_FIRST_v = ID_FIRST_steer + N;
const int ID_FIRST_steer_ac = ID_FIRST_v + N;
const int ID_FIRST_v_ac = ID_FIRST_steer_ac + N - 1;

const double W_cte = 1.0;
const double W_eheading = 0.0;
const double W_v = 0.0;
const double W_steer_ac = 0.0;
const double W_v_ac = 0.0;
const double W_dv_ac = 0.0;
const double W_dsteer_ac = 0.0;
#endif // MPC_PRIVATE

typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC
{
    public:
        MPC(ForkliftModel<double>& model);

        void solve(const Eigen::VectorXd& K, double desiredSpeed);

        void draw(sf::RenderWindow& window, TransformStack& ts);

        ForkliftModel<double>& model;

        double steer_ac;
        double speed_ac;

        Dvector x;
        Dvector xLowerBound;
        Dvector xUpperBound;
        Dvector gLowerBound;
        Dvector gUpperBound;

        std::vector<Eigen::Vector2d> prediction;

    protected:

    private:
};

#endif // MPC_H
