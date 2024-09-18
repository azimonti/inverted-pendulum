#ifndef _PENDULUM_DATA_H_FC0F6FA76C0549039F3F00F5F07B59A6_
#define _PENDULUM_DATA_H_FC0F6FA76C0549039F3F00F5F07B59A6_

/************************/
/*   pendulum_data.h    */
/*    Version 1.0       */
/*     2023/04/14       */
/*  © Marco Azimonti    */
/************************/

#include <array>
#include <vector>

namespace physics
{

#define T_C(x) static_cast<T>(x)
#define T_PI   static_cast<T>(3.14159265358979323846)

    template <typename T> struct ExternalParams
    {
        // gravity
        const T g = T_C(9.80665);
        // damping
#ifdef WITH_DAMPING
        const T Damping = T_C(0.4);
#else
        const T Damping = T_C(0.0);
#endif
    };

    template <typename T> struct Body
    {
        // length 2.0 m
        const T length = T_C(2.0);
        // mass 1.0 kg
        const T mass   = T_C(1.0);
    };

    template <typename T> struct OdeParams
    {
        // initial angle
        const T theta     = T_C(2.7);
        // initial angular velocity
        const T theta_dot = T_C(0.5);
    };

} // namespace physics

namespace control
{
    template <typename T> struct Params
    {
        // control gain
        const T KGainNN        = T_C(100.0);
        // linearization point
        std::array<T, 2> y_ref = {T_PI, 0};
        // Target Eigenvalues
        std::array<T, 2> eigen{-1.0, -3.0};
    };
} // namespace control

namespace nn
{
    struct NNParams
    {
        // hidden layer structure
        const std::vector<size_t> hlayers = {8, 4};
        // seed
        const int seed                    = 4247;
        // number of top individuals to be selected
        const size_t top_individuals      = 10;
        // population size
        const size_t population_size      = 300;
        // number of inputs
        const size_t inputs               = 3;
        // use neural network
        const bool use_nn                 = true;
        // always start from the same initial conditions
        const bool fixed_ic               = true;
        // number of epochs to Train in a single run
        const size_t epochsThread         = 100;
        // number of threads to use
        const size_t nthreads             = 6;
    };
} // namespace nn

#undef T_PI
#undef T_C

#endif
