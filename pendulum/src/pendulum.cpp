/************************/
/*    pendulum.cpp      */
/*    Version 1.0       */
/*     2023/04/13       */
/*  © Marco Azimonti    */
/************************/

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <numeric>
#include <thread>
#include "log/log.h"
#include "solvers/runge_kutta.h"
#include "thread/thread_pool.hpp"
#include "pendulum.h"
#include "pendulum_data.h"

#define T_C(x) static_cast<T>(x)
#define T_PI   static_cast<T>(3.14159265358979323846)

namespace ge
{
    template <typename T> std::string num_to_string(T num, int decimals = 2)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(decimals) << num;
        return ss.str();
    }

    template <typename T>
    std::array<T, 2>& motionFun(T t, const std::array<T, 2>& y, std::array<T, 2>& dydt,
                                const math::ode::Params<T>& params)
    {
        (void)t;
        dydt[0] = y[1];
        dydt[1] = -params.omega2 * sin(y[0]) - params.gamma * y[1] + params.u;
        return dydt;
    }

} // namespace ge

template <typename T>
ge::Pendulum<T>::Pendulum()
    : length(T_C(1)), mass(T_C(1)), theta(T_C(0)), thetaDot(T_C(0)), thetaDDot(T_C(0)), time(T_C(0)),
      deltaTime(T_C(0.025)), bestFitness(T_C(0)), flags(pflags::GRID & pflags::F2D)
{
    if (nn::NNParams().use_nn) flags = (flags | pflags::USE_NN);
};

template <typename T>
ge::Pendulum<T>::Pendulum(T length, T mass)
    : length(length), mass(mass), theta(T_C(0)), thetaDot(T_C(0)), thetaDDot(T_C(0)), time(T_C(0)),
      deltaTime(T_C(0.025)), bestFitness(T_C(0)), flags(pflags::GRID & pflags::F2D)
{
    if (nn::NNParams().use_nn) flags = (flags | pflags::USE_NN);
};

template <typename T> void ge::Pendulum<T>::onInit()
{
    // set pendulum parameters
    length        = physics::Body<T>().length;
    mass          = physics::Body<T>().mass;
    // set non-dimensional parameters
    params.omega2 = physics::ExternalParams<T>().g / length;
    params.gamma  = physics::ExternalParams<T>().Damping / mass;
    // set initial conditions
    Reset();

    T pendulum_up = T_C(1); // 1 for up, -1 for down

    A             = la::Matrix<T>(std::vector<T>{0, 1, pendulum_up * params.omega2, -params.gamma}, 2, 2);
    B             = la::Matrix<T>(std::vector<T>{0, 1}, 2, 1);
    // K is analytically computed
    K = la::Matrix<T>(std::vector<T>{control::Params<T>().eigen[0] * control::Params<T>().eigen[1] + params.omega2,
                                     -(params.gamma + control::Params<T>().eigen[0] + control::Params<T>().eigen[1])},
                      2, 1);
    KGainNN = control::Params<T>().KGainNN;
    (void)A;
    (void)B;

    if (pflags::USE_NN) InitNN();
}

template <typename T> void ge::Pendulum<T>::onUpdate()
{
    // launch TrainNN on a separate thread and check completion to avoid multiple thread
    if ((flags & pflags::NN_TRAIN) && (flags & pflags::ASYNC))
    {
        if (!isTraining)
        {
            if (flags & pflags::VERBOSE)
                LOGGER(logging::INFO) << std::string("Training in progress for " +
                                                     num_to_string(nn::NNParams().epochsThread) + " epochs...");
            mFuture = std::async(std::launch::async, &ge::Pendulum<T>::MultipleEpochNN, this);
        }
    }
    else if (flags & pflags::NN_TRAIN) TrainNN();
    ComputeMotion();
}

template <typename T> void ge::Pendulum<T>::PrintState()
{
    LOGGER(logging::INFO) << std::string(
        "time : " + num_to_string(time) + "    theta : " + num_to_string(theta) +
        "    theta_dot : " + num_to_string(thetaDot) + "    theta_ddot : " + num_to_string(thetaDDot) +
        "    current generation : " + num_to_string(nGenerations) + "    best fitness : " + num_to_string(bestFitness));
}

template <typename T> void ge::Pendulum<T>::PrintExtendedState()
{
    std::vector<T> nn_out(2);
    ComputeControlNN(nn_out);

    LOGGER(logging::INFO) << std::string(
        "time : " + num_to_string(time) + " theta : " + num_to_string(theta) +
        " theta_dot : " + num_to_string(thetaDot) + " theta_ddot : " + num_to_string(thetaDDot) +
        " gen : " + num_to_string(nGenerations) + " fit : " + num_to_string(bestFitness) + " K: [" +
        num_to_string(K[0][0]) + " " + num_to_string(K[1][0]) + "]" + " K_NN: [" + num_to_string(KGainNN * nn_out[0]) +
        " " + num_to_string(KGainNN * nn_out[1]) + "]");
}

template <typename T> void ge::Pendulum<T>::Reset()
{
    theta     = physics::OdeParams<T>().theta;
    thetaDot  = physics::OdeParams<T>().theta_dot;
    thetaDDot = -params.omega2 * sin(theta) - params.gamma * thetaDot;
    time      = T_C(0);
}

template <typename T> void ge::Pendulum<T>::InitNN()
{
    // set neural network parameters
    std::vector<size_t> nnsize;
    // number of inputs
    nnsize.push_back(nn::NNParams().inputs);
    for (size_t i = 0; i < nn::NNParams().hlayers.size(); ++i) nnsize.push_back(nn::NNParams().hlayers[i]);
    // number of outputs
    nnsize.push_back(2);
    mNN = std::make_unique<nn::ANN_MLP_GA<T>>(nnsize, nn::NNParams().seed, nn::NNParams().population_size,
                                              nn::NNParams().top_individuals, nn::TANH);
    mNN->SetName("pendulum");
    mNN->CreatePopulation();
}

template <typename T> void ge::Pendulum<T>::TrainNN()
{
    assert(mNN != nullptr);
    // Reset the pendulum to the initial conditions and velocity or use the current position and velocity
    const T theta0_     = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta : theta;
    const T thetaDot0_  = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta_dot : thetaDot;
    const T thetaDDot0_ = -params.omega2 * sin(theta0_) - params.gamma * thetaDot0_;

    if (flags & pflags::VERBOSE)
        LOGGER(logging::INFO) << std::string("Running generation " + num_to_string(nGenerations) + "...");

    const size_t pSize = mNN->GetPopSize();
    std::vector<T> fitness(pSize);
    std::vector<size_t> idx(pSize);

    // compute the motion for 10 seconds
    const size_t simSteps = static_cast<size_t>(T_C(10) / deltaTime);
    tp::thread_pool pool;
    for (size_t i = 0; i < pSize; ++i)
    {
        auto floop = [this, &fitness = fitness[i], idx = i, &theta0_, &thetaDot0_, &thetaDDot0_, &simSteps]() {
            std::array<T, 2> y1_, y2_;
            math::ode::Params<T> params_copy = params;

            std::vector<T> inputs(nn::NNParams().inputs), nn_out(2);
            T t_ = T_C(0.0), theta_ = theta0_, thetaDot_ = thetaDot0_, thetaDDot_ = thetaDDot0_;
            for (size_t j = 0; j < simSteps; ++j)
            {
                inputs[0] = theta_ / T_C(2.0 * T_PI);
                inputs[1] = thetaDot_ / T_C(2.0 * T_PI);
                inputs[2] = thetaDDot_ / T_C(2.0 * T_PI);
                mNN->feedforward(inputs, nn_out, idx, false);

                params_copy.u = -KGainNN * (nn_out[0] * (theta_ - control::Params<T>().y_ref[0]) +
                                            nn_out[1] * (thetaDot_ - control::Params<T>().y_ref[1]));
                y1_           = {theta_, thetaDot_};
                ma::rk4singlestep(ge::motionFun, deltaTime, t_, y1_, y2_, params_copy);
                theta_    = y2_[0];
                thetaDot_ = y2_[1];
                // run the function again to get the correct thetaDDot y2_ is now the good input, y1_ is the output
                ge::motionFun(time, y2_, y1_, params_copy);
                thetaDDot_ = y1_[1];
                fitness += std::abs(theta_ - control::Params<T>().y_ref[0]);
                fitness += std::abs(thetaDot_ - control::Params<T>().y_ref[1]);
                t_ += deltaTime;
            }
            fitness /= T_C(simSteps);
        };

        if (flags & pflags::MULTITHREAD) { pool.push_task(floop); }
        else floop();
    }
    pool.wait_for_tasks();
    //  sort in ascending order and store the index
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(idx.begin(), idx.end(), [&fitness](size_t i1, size_t i2) { return fitness[i1] < fitness[i2]; });
    mNN->UpdateWeightsAndBiases(idx);
    mNN->CreatePopulation();
    bestFitness = fitness[idx[0]];
    if (flags & pflags::VERBOSE)
    {
        for (size_t i = 0; i < nn::NNParams().top_individuals; ++i)
            LOGGER(logging::INFO) << std::string("Top " + num_to_string(i) + " : " + num_to_string(idx[i]) +
                                                 " fitness : " + num_to_string(fitness[idx[i]]));
        std::vector<T> nn_out(2);
        ComputeControlNN(nn_out);

        // print the best performer fitness
        LOGGER(logging::INFO) << std::string(
            "Best performer fitness : " + num_to_string(fitness[idx[0]]) + num_to_string(bestFitness) + " K: [" +
            num_to_string(K[0][0]) + " " + num_to_string(K[1][0]) + "]" + " K_NN: [" +
            num_to_string(KGainNN * nn_out[0]) + " " + num_to_string(KGainNN * nn_out[1]) + "]");
    }
    nGenerations++;
    mNN->UpdateEpochs();
}

template <typename T> void ge::Pendulum<T>::MultipleEpochNN()
{
    isTraining = true;
    for (size_t i = 0; i < nn::NNParams().epochsThread; ++i) TrainNN();
    isTraining = false;
}

template <typename T> std::vector<T>& ge::Pendulum<T>::ComputeControlNN(std::vector<T>& nn_out)
{
    std::vector<T> inputs(nn::NNParams().inputs);
    inputs[0] = theta / T_C(2.0 * T_PI);
    inputs[1] = thetaDot / T_C(2.0 * T_PI);
    inputs[2] = thetaDDot / T_C(2.0 * T_PI);
    mNN->feedforward(inputs, nn_out, 0, false);
    return nn_out;
}

template <typename T> void ge::Pendulum<T>::ComputeMotion()
{
    std::array<T, 2> y1_ = {theta, thetaDot}, y2_;
    params.u             = 0;

    // use the neural network to compute the control signal
    if (flags & pflags::CONTROL_NN)
    {
        // compute the control signal
        std::vector<T> nn_out(2);
        ComputeControlNN(nn_out);
        params.u = -KGainNN * (nn_out[0] * (theta - control::Params<T>().y_ref[0]) +
                               nn_out[1] * (thetaDot - control::Params<T>().y_ref[1]));
    }
    // use the LQR to compute the control signal
    else if (flags & pflags::CONTROL)
        for (size_t i = 0; i < 2; i++) params.u -= K[i][0] * (y1_[i] - control::Params<T>().y_ref[i]);

    ma::rk4singlestep(ge::motionFun, deltaTime, time, y1_, y2_, params);

    theta    = y2_[0];
    thetaDot = y2_[1];
    // run the function again to get the correct thetaDDot y2_ is now the good input, y1_ is the output
    ge::motionFun(time, y2_, y1_, params);
    thetaDDot = y1_[1];

    time += deltaTime;
    frames++;
    if (flags & pflags::VVERBOSE) PrintState();
}

#undef T_C
#undef T_PI

// template instantiation
template class ge::Pendulum<float>;
template class ge::Pendulum<double>;
