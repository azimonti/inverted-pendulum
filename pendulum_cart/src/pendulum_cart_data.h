#ifndef _PENDULUM_CART_DATA_H_FC0F6FA76C0549039F3F00F5F07B59A6_
#define _PENDULUM_CART_DATA_H_FC0F6FA76C0549039F3F00F5F07B59A6_

/************************/
/* pendulum_cart_data.h */
/*    Version 1.0       */
/*     2023/04/14       */
/************************/

#include <array>
#include <cmath>

namespace physics
{

#define T_C(x) static_cast<T>(x)
#define T_PI   static_cast<T>(3.14159265358979323846)

    template <typename T> struct ExternalParams
    {
        // gravity
        const T g = T_C(9.80665);
#ifdef UP_POSITION
        // cart damping
        const T Damping = T_C(0.75);
#else
        // no damping
        const T Damping              = T_C(0.0);
#endif
    };

    template <typename T> struct Body
    {
        // length 1.5 m
        const T length = T_C(1.5);
        // mass of pendulum 1.5 kg
        const T mass1  = T_C(1.0);
        // mass of cart 5 kg
        const T mass2  = T_C(5.0);
    };

    template <typename T> struct OdeParams
    {
        // pendulum
        // initial angle
        const T theta     = T_PI;
        // initial angular velocity
        const T theta_dot = T_C(0.5);
        // cart
        // initial position
        const T x         = T_C(0.0);
        // initial velocity
        const T x_dot     = T_C(0.0);
    };
} // namespace physics

namespace control
{
    template <typename T> struct Params
    {
        // control gain
        const T nnKGain = T_C(100.0);
#ifdef UP_POSITION
        // linearization point
        const std::array<T, 4> y_ref = {2.0, 0, T_PI, 0};
        // coefficient of the linearization
        const int a_coeff            = 1;
        const std::vector<T> KGain   = {T_C(-0.2650), T_C(-2.1939), T_C(92.1907), T_C(26.1659)};
#else
        // linearization point
        const std::array<T, 4> y_ref = {2.0, 0, 0, 0};
        const int a_coeff            = -1;
        const std::vector<T> KGain   = {T_C(0.2650), T_C(1.4439), T_C(36.0907), T_C(-21.8341)};
#endif
    };

} // namespace control

namespace nn
{
    struct NNParams
    {
        // nn name
        const std::string name            = "invpendulum";
        // hidden layer structure
        const std::vector<size_t> hlayers = {8, 16, 8};
        // seed
        const int seed                    = 5247;
        // number of top individuals to be selected
        const size_t top_individuals      = 10;
        // population size
        const size_t population_size      = 300;
        // use mixed population
        const bool mixed_population       = true;
        // keep the best individual as-is
        const bool elitism                = true;
        // number of inputs
        const size_t inputs               = 4;
        // use neural network
        const bool use_nn                 = true;
        // simulation time
        const double sim_time             = 20.0;
        // always start from the same initial conditions
        const bool fixed_ic               = true;
        // number of epochs to Train in a single run
        const size_t epochsThread         = 100;
        // save
        const bool save_nn                = true;
        // save overwriting
        const bool overwrite              = false;
        // save path
        const std::string save_path       = "./build/archive/";
        // save every n generations
        const size_t save_every           = 250;
    };
} // namespace nn

#undef T_PI
#undef T_C

#endif
