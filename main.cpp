
#ifdef asd
#include <cppad/ipopt/solve.hpp>
#include <iostream>

namespace {
     using CppAD::AD;

     class FG_eval {
     public:
          typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
          void operator()(ADvector& fg, const ADvector& x)
          {
              std::cout << "fg_eval" << std::endl;

              // Fortran style indexing
               AD<double> x1 = x[0];
               AD<double> x2 = x[1];
               AD<double> x3 = x[2];
               AD<double> x4 = x[3];
               // f(x)
               fg[0] = x1 * x4 * (x1 + x2 + x3) + x3;
               // g_1 (x)
               fg[1] = x1 * x2 * x3 * x4;
               // g_2 (x)
               fg[2] = x1 * x1 + x2 * x2 + x3 * x3 + x4 * x4;

               std::cout << "fg: " << fg << std::endl;
               std::cout << "x: " << x << std::endl;
          }
     };
}

int main()
{
    bool ok = true;
     size_t i;
     typedef CPPAD_TESTVECTOR( double ) Dvector;

     // number of independent variables (domain dimension for f and g)
     size_t nx = 4;
     // number of constraints (range dimension for g)
     size_t ng = 2;
     // initial value of the independent variables
     Dvector xi(nx);
     xi[0] = 1.0;
     xi[1] = 5.0;
     xi[2] = 5.0;
     xi[3] = 1.0;
     // lower and upper limits for x
     Dvector xl(nx), xu(nx);
     for(i = 0; i < nx; i++)
     {     xl[i] = 1.0;
          xu[i] = 5.0;
     }
     // lower and upper limits for g
     Dvector gl(ng), gu(ng);
     gl[0] = 25.0;     gu[0] = 1.0e19;
     gl[1] = 40.0;     gu[1] = 40.0;

     // object that computes objective and constraints
     FG_eval fg_eval;

     // options
     std::string options;
     options += "Retape  true\n";
     // turn off any printing
     options += "Integer print_level  0\n";
     options += "String  sb           yes\n";
     // maximum number of iterations
     options += "Integer max_iter     10\n";
     // approximate accuracy in first order necessary conditions;
     // see Mathematical Programming, Volume 106, Number 1,
     // Pages 25-57, Equation (6)
     options += "Numeric tol          1e-6\n";
     // derivative testing
     options += "String  derivative_test            second-order\n";
     // maximum amount of random pertubation; e.g.,
     // when evaluation finite diff
     options += "Numeric point_perturbation_radius  0.\n";

     // place to return solution
     CppAD::ipopt::solve_result<Dvector> solution;

     // solve the problem
     CppAD::ipopt::solve<Dvector, FG_eval>(
          options, xi, xl, xu, gl, gu, fg_eval, solution
     );
     //
     // Check some of the solution values
     //
     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
     //
     double check_x[]  = { 1.000000, 4.743000, 3.82115, 1.379408 };
     double check_zl[] = { 1.087871, 0.,       0.,      0.       };
     double check_zu[] = { 0.,       0.,       0.,      0.       };
     double rel_tol    = 1e-6;  // relative tolerance
     double abs_tol    = 1e-6;  // absolute tolerance
     for(i = 0; i < nx; i++)
     {     ok &= CppAD::NearEqual(
               check_x[i],  solution.x[i],   rel_tol, abs_tol
          );
          ok &= CppAD::NearEqual(
               check_zl[i], solution.zl[i], rel_tol, abs_tol
          );
          ok &= CppAD::NearEqual(
               check_zu[i], solution.zu[i], rel_tol, abs_tol
          );
     }

     std::cout << "ok: " << ok << std::endl;

     std::cout << "solution: " << solution.x << std::endl;

     system("pause");

     return 0;
}

#endif

#include <SFML/Graphics.hpp>
#include "Forklift.h"
#include "Frame.h"
#include <iostream>
#include "BeizerCurve.h"
#include "NavError.h"
#include "MPC.h"
#include "Utils.h"

const int WINDOW_WIDTH = 960;
const int WINDOW_HEIGHT = 600;

int main()
{
    bool automatic = false;

    Forklift forklift;
    forklift.model.position[0] = 1;
    forklift.model.position[1] = 2.8;
    forklift.steer = 0.01;

    MPC mpc(forklift.model);

    Frame globalFrame;
    Frame targetFrame;

    BeizerCurve beizer1;
    beizer1.add(Eigen::Vector2d(1, 3));
    beizer1.add(Eigen::Vector2d(3, 3));
    beizer1.add(Eigen::Vector2d(3, 1));

    BeizerCurve beizer2;
    beizer2.add(Eigen::Vector2d(3, 1));
    beizer2.add(Eigen::Vector2d(3, 0));
    beizer2.add(Eigen::Vector2d(2, 0));
    beizer2.add(Eigen::Vector2d(2, -1));

    BeizerCurve beizer3;
    beizer3.add(Eigen::Vector2d(2, -1));
    beizer3.add(Eigen::Vector2d(2, -3));

    BeizerCurve beizer4;
    beizer4.add(Eigen::Vector2d(2, -3));
    beizer4.add(Eigen::Vector2d(2, -5));
    beizer4.add(Eigen::Vector2d(0, -5));

    std::vector<Eigen::Vector2d> path;
    std::vector<Eigen::Vector2d> curve;
    std::vector<Eigen::Vector2d> segment;

    curve = beizer1.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer2.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer3.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());
    curve = beizer4.getCurve();
    path.insert(path.end(), curve.begin(), curve.end());

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "MPC", sf::Style::Default, settings);

    TransformStack ts;
    ts.rotate(-90);
    ts.scale(1, -1);
    ts.translate(WINDOW_HEIGHT/-2, WINDOW_WIDTH/-2);
    ts.scale(70, 70);

    sf::Clock clock;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed) {
                window.close();

            } else if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2f point = static_cast<sf::Transform>(ts).getInverse().transformPoint(event.mouseButton.x, event.mouseButton.y);
                    targetFrame.pos = Eigen::Vector2d(point.x, point.y);
                    //beizer1.add(targetFrame.pos);
                }

            } else if (event.type == sf::Event::KeyReleased) {
                if (event.key.code == sf::Keyboard::A) {
                    automatic = !automatic;
                }
            }
        }

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) forklift.driveCmd = FORWARD;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) forklift.driveCmd = BACKWARD;
        else forklift.driveCmd = STOP;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) forklift.steerCmd = RIGHT;
        else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) forklift.steerCmd = LEFT;
        else forklift.steerCmd = NONE;

        float dt = clock.restart().asSeconds();

        /*Frame f;
        f.pos = forklift.model.position;
        f.angle = forklift.model.heading;
        NavError navError = NavError::calcNavError(curve, f);*/

        segment = transformPointsIntoFrame(path, forklift.model.position, forklift.model.heading);
        segment = rollingWindowPath(segment, 0.2, 2.0);

        const int ORDER = 3;
        Eigen::VectorXd K = polyfit(segment, ORDER);

        std::vector<Eigen::Vector2d> fitted;
        for (int i = -50; i < 50; ++i) {
            const double dx = 0.1 * i;
            const double dy = K[3] * dx * dx * dx + K[2] * dx * dx + K[1] * dx + K[0];
            fitted.push_back(Eigen::Vector2d(dx, dy));
        }

        mpc.solve(K, forklift.model.MAX_SPEED);

        std::cout << mpc.speed_ac << ", " << mpc.steer_ac << std::endl;

        if (automatic) {
            forklift.speed = mpc.speed_ac * 0.9;
            //forklift.steer = -20 * M_PI / 180;
        } else {
            forklift.control(dt);
        }

        forklift.update(dt);

        window.clear(sf::Color::White);

        globalFrame.draw(window, ts);
        targetFrame.draw(window, ts);

        beizer1.draw(window, ts);
        beizer2.draw(window, ts);
        beizer3.draw(window, ts);
        beizer4.draw(window, ts);

        //navError.draw(window, ts);

        forklift.draw(window, ts);

        mpc.draw(window, ts);

        drawPath(window, ts, fitted, sf::Color::Cyan);
        drawPath(window, ts, segment, sf::Color::Magenta);

        window.display();
    }

    return 0;
}
