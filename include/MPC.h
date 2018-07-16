#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Forklift.h"

// Define this if you want to use an approximate model in the MPC algorithm
// --> #define MPC_APPROXIMATE


#ifdef MPC_PRIVATE // This is set in MPC.cpp to make definitions only accessible to that file

 // N - is how many states we "lookahead" in the future

// Can use longer horizon with worse accuracy when using the approximate model
#ifdef MPC_APPROXIMATE
const double DT = 0.1;
const int N = 10;
#else
// We need shorter horizon if we don't use approximate model
const double DT = 0.2;
const int N = 5;
#endif // MPC_APPROXIMATE

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

const double W_cte = 1000.0;
const double W_eheading = 150.0;
const double W_v = 50.0;
const double W_v_stop = 2000.0;
const double W_steer = 0.4;
const double W_steer_ac = 0.2;
const double W_v_ac = 0.2;
const double W_dv_ac = 0.1;
const double W_dsteer_ac = 0.1;
#endif // MPC_PRIVATE

typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC
{
    public:
        MPC(ForkliftModel<double>& model);

        void solve(const Eigen::VectorXd& K, double desiredSpeed, double distanceLeft = -1);

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
