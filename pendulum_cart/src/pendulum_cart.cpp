/************************/
/*  pendulum_cart.cpp   */
/*    Version 1.0       */
/*     2023/04/13       */
/************************/

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include "log/log.h"
#include "solvers/runge_kutta.h"
#include "thread/thread_pool.hpp"
#include "pendulum_cart.h"
#include "pendulum_cart_data.h"

#define T_C(x)   static_cast<T>(x)
#define T_PI     static_cast<T>(3.14159265358979323846)
#define XDIVISOR static_cast<T>(8 * 3.14159265358979323846)
#define TDIVISOR static_cast<T>(4 * 3.14159265358979323846)

namespace ge
{
    template <typename T> std::string num_to_string(T num, int decimals = 2)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(decimals) << num;
        return ss.str();
    }

    template <typename T>
    std::array<T, 4>& motionFun2(T t, const std::array<T, 4>& y, std::array<T, 4>& dydt,
                                 const math::ode::Params<T>& params)
    {
        (void)t;

        const T Sx = sin(y[2]);
        const T Cx = cos(y[2]);
        const T D  = params.m * params.L * params.L * (params.M + params.m * (1 - Cx * Cx));

        dydt[0]    = y[1];
        dydt[1] =
            (1 / D) * (params.m * params.m * params.L * params.L * params.g * Cx * Sx +
                       params.m * params.L * params.L * (params.m * params.L * y[3] * y[3] * Sx - params.b * y[1])) +
            params.m * params.L * params.L * (1 / D) * params.u;
        dydt[2] = y[3];
        dydt[3] = (1 / D) * (-(params.m + params.M) * params.m * params.g * params.L * Sx -
                             params.m * params.L * Cx * (params.m * params.L * y[3] * y[3] * Sx - params.b * y[1])) -
                  params.m * params.L * Cx * (1 / D) * params.u;

        return dydt;
    }

    template <typename T>
    std::array<T, 4>& motionFun(T t, const std::array<T, 4>& y, std::array<T, 4>& dydt,
                                const math::ode::Params<T>& params)
    {
        (void)t;

        const T Sx = sin(y[2]);
        const T Cx = cos(y[2]);
        const T D  = 1 / (params.L * (params.M + params.m * (1 - Cx * Cx)));

        dydt[0]    = y[1];
        dydt[1]    = params.L * D *
                  (params.m * params.g * Cx * Sx + params.m * params.L * Sx * y[3] * y[3] - params.b * y[1] + params.u);
        dydt[2] = y[3];
        dydt[3] = D * (-(params.m + params.M) * params.g * Sx -
                       Cx * (params.m * params.L * Sx * y[3] * y[3] - params.b * y[1] + params.u));

        return dydt;
    }

} // namespace ge

template <typename T>
ge::InvertedPendulum<T>::InvertedPendulum()
    : length(T_C(1)), mass1(T_C(1)), theta(T_C(0)), thetaDot(T_C(0)), mass2(T_C(1)), x(T_C(0)), xDot(T_C(0)),
      time(0.0f), deltaTime(0.025f), bestFitness(T_C(0)), flags(pflags::GRID | pflags::F2D | pflags::CONTROL),
      kGain(T_C(1)), nnK(4)
{

    if (nn::NNParams().use_nn) flags = (flags | pflags::USE_NN);
};

template <typename T>
ge::InvertedPendulum<T>::InvertedPendulum(T length, T mass1, T mass2)
    : length(length), mass1(mass1), theta(T_C(0)), thetaDot(T_C(0)), mass2(mass2), x(T_C(0)), xDot(T_C(0)), time(0.0f),
      deltaTime(0.025f), bestFitness(T_C(0)), flags(pflags::GRID | pflags::F2D | pflags::CONTROL), kGain(T_C(1)), nnK(4)
{
    if (nn::NNParams().use_nn) flags = (flags | pflags::USE_NN);
};

template <typename T> void ge::InvertedPendulum<T>::onInit()
{
    // set pendulum parameters
    length   = physics::Body<T>().length;
    mass1    = physics::Body<T>().mass1;
    mass2    = physics::Body<T>().mass2;

    // set ode parameters
    params.m = mass1;
    params.M = mass2;
    params.L = length;
    params.g = physics::ExternalParams<T>().g;
    params.b = physics::ExternalParams<T>().Damping;

    // set initial conditions
    Reset();

    T pendulum_up = physics::ExternalParams<T>().g; // 1 for up, -1 for down
    A = la::Matrix<T>(std::vector<T>{0, 1, 0, 0, 0, -params.b / params.M, params.m * params.g / params.M, 0, 0, 0, 0, 1,
                                     0, -1 * pendulum_up * params.b / (params.M * params.L),
                                     pendulum_up * (params.m + params.M) * params.g / (params.M * params.L), 0},
                      4, 4);
    B = la::Matrix<T>(std::vector<T>{0, 1 / params.M, 0, pendulum_up * 1 / (params.M * params.L)});
    K = la::Matrix<T>(control::Params<T>().KGain, 4, 1);
    nnKGain = control::Params<T>().nnKGain;
    (void)A;
    (void)B;
    if (pflags::USE_NN) InitNN();
}

template <typename T> void ge::InvertedPendulum<T>::Reset()
{
    x        = physics::OdeParams<T>().x;
    xDot     = physics::OdeParams<T>().x_dot;
    theta    = physics::OdeParams<T>().theta;
    thetaDot = physics::OdeParams<T>().theta_dot;
    time     = T_C(0);
}

template <typename T> void ge::InvertedPendulum<T>::InitNN()
{
    // set neural network parameters
    std::vector<size_t> nnsize;
    // number of inputs
    nnsize.push_back(nn::NNParams().inputs);
    for (size_t i = 0; i < nn::NNParams().hlayers.size(); ++i) nnsize.push_back(nn::NNParams().hlayers[i]);
    // number of outputs
    nnsize.push_back(4);
    mNN = std::make_unique<nn::ANN_MLP_GA<T>>(nnsize, nn::NNParams().seed, nn::NNParams().population_size,
                                              nn::NNParams().top_individuals, nn::TANH);
    mNN->SetName(nn::NNParams().name);

    mNN->SetMixed(nn::NNParams().mixed_population);
    mNN->CreatePopulation(nn::NNParams().elitism);

    // create save directory if not exists
    if (nn::NNParams().save_nn)
        if (!std::filesystem::exists(nn::NNParams().save_path))
            std::filesystem::create_directory(nn::NNParams().save_path);
}

template <typename T> void ge::InvertedPendulum<T>::onUpdate()
{
    // launch TrainNN on a separate thread and check completion to avoid multiple thread
    if ((flags & pflags::NN_TRAIN) && (flags & pflags::ASYNC))
    {
        if (!isTraining)
        {
            if (flags & pflags::VERBOSE)
                LOGGER(logging::INFO) << std::string("Training in progress for " +
                                                     num_to_string(nn::NNParams().epochsThread) + " epochs...");
            mFuture = std::async(std::launch::async, &InvertedPendulum::MultipleEpochNN, this);
        }
    }
    else if (flags & pflags::NN_TRAIN) TrainNN();
    ComputeMotion();
    IncrementTime();
}

template <typename T> void ge::InvertedPendulum<T>::PrintState()
{
    LOGGER(logging::INFO) << std::string("time : " + num_to_string(time) + "    theta : " + num_to_string(theta) +
                                         "    theta_dot : " + num_to_string(thetaDot) + "    x : " + num_to_string(x) +
                                         "    x_dot : " + num_to_string(xDot));
}

template <typename T> void ge::InvertedPendulum<T>::PrintExtendedState()
{
    const T x0_        = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().x : x;
    const T xDot0_     = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().x_dot : xDot;
    const T theta0_    = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta : theta;
    const T thetaDot0_ = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta_dot : thetaDot;
    std::vector<T> inputs_(nn::NNParams().inputs), nn_out_(4);
    inputs_[0] = x0_ / XDIVISOR;
    inputs_[1] = xDot0_ / XDIVISOR;
    inputs_[2] = theta0_ / TDIVISOR;
    inputs_[3] = thetaDot0_ / TDIVISOR;
    mNN->feedforward(inputs_, nn_out_, 0, false);

    LOGGER(logging::INFO) << std::string(
        "time : " + num_to_string(time) + " theta : " + num_to_string(theta) +
        " theta_dot : " + num_to_string(thetaDot) + " x :" + num_to_string(x) + " x_dot" + num_to_string(xDot) +
        " gen : " + num_to_string(nGenerations) + " fit : " + num_to_string(bestFitness) + " K: [" +
        num_to_string(K[0][0]) + " " + num_to_string(K[1][0]) + " " + num_to_string(K[2][0]) + " " +
        num_to_string(K[3][0]) + "]" + " K_NN: [" + num_to_string(nnKGain * nn_out_[0]) + " " +
        num_to_string(nnKGain * nn_out_[1]) + " " + num_to_string(nnKGain * nn_out_[2]) + " " +
        num_to_string(nnKGain * nn_out_[3]) + "]");
}

template <typename T> T ge::InvertedPendulum<T>::ComputeBestFitness()
{
    assert(mNN != nullptr);
    // Reset the pendulum to the initial conditions and velocity or use the current position and velocity
    T x_        = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().x : x;
    T xDot_     = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().x_dot : xDot;
    T theta_    = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta : theta;
    T thetaDot_ = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta_dot : thetaDot;
    T t_        = T_C(0.0);
    std::vector<T> inputs_(nn::NNParams().inputs), nn_out_(4);
    std::array<T, 4> y1_, y2_;
    const size_t simSteps = static_cast<size_t>(T_C(nn::NNParams().sim_time) / deltaTime);
    bestFitness           = T_C(0.0);
    for (size_t j = 0; j < simSteps; ++j)
    {
        inputs_[0] = x_ / XDIVISOR;
        inputs_[1] = xDot_ / XDIVISOR;
        inputs_[2] = theta_ / TDIVISOR;
        inputs_[3] = thetaDot_ / TDIVISOR;

        // compute the output of the NN - 0 is the best performer
        mNN->feedforward(inputs_, nn_out_, 0, false);

        params.u = -nnKGain * (nn_out_[0] * (x_ - control::Params<T>().y_ref[0]) +
                               nn_out_[1] * (xDot_ - control::Params<T>().y_ref[1]) +
                               nn_out_[2] * (theta_ - control::Params<T>().y_ref[2]) +
                               nn_out_[3] * (thetaDot_ - control::Params<T>().y_ref[3]));
        y1_      = {x_, xDot_, theta_, thetaDot_};
        ma::rk4singlestep(ge::motionFun, deltaTime, t_, y1_, y2_, params);
        x_        = y2_[0];
        xDot_     = y2_[1];
        theta_    = y2_[2];
        thetaDot_ = y2_[3];
        // check if too big or too small
        if (x_ > T_C(1000) || x_ < -T_C(1000) || theta_ > T_C(1000) || theta_ < -T_C(1000))
        {
            bestFitness = std::numeric_limits<T>::max();
            return bestFitness;
        }

        bestFitness += std::abs(x_ - control::Params<T>().y_ref[0]);
        bestFitness += std::abs(xDot_ - control::Params<T>().y_ref[1]);
        bestFitness += T_C(5) * std::abs(theta_ - control::Params<T>().y_ref[2]);
        bestFitness += std::abs(thetaDot_ - control::Params<T>().y_ref[3]);
        t_ += deltaTime;
    }
    bestFitness /= T_C(simSteps);
    return bestFitness;
}

template <typename T> void ge::InvertedPendulum<T>::TrainNN()
{
    assert(mNN != nullptr);
    // Reset the pendulum to the initial conditions and velocity or use the current position and velocity
    const T x0_        = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().x : x;
    const T xDot0_     = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().x_dot : xDot;
    const T theta0_    = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta : theta;
    const T thetaDot0_ = (nn::NNParams().fixed_ic) ? physics::OdeParams<T>().theta_dot : thetaDot;

    if (flags & pflags::VERBOSE)
        LOGGER(logging::INFO) << std::string("Running generation " + num_to_string(nGenerations) + "...");

    const size_t pSize = mNN->GetPopSize();
    std::vector<T> fitness(pSize);
    std::vector<size_t> idx(pSize);
    const size_t simSteps = static_cast<size_t>(T_C(nn::NNParams().sim_time) / deltaTime);
    tp::thread_pool pool;
    for (size_t i = 0; i < pSize; ++i)
    {
        auto floop = [this, &fitness = fitness[i], idx = i, &x0_, &xDot0_, &theta0_, &thetaDot0_, &simSteps]() {
            std::array<T, 4> y1_, y2_;
            math::ode::Params<T> params_copy = params;

            std::vector<T> inputs_(nn::NNParams().inputs), nn_out_(4);
            T t_ = T_C(0.0), x_ = x0_, xDot_ = xDot0_, theta_ = theta0_, thetaDot_ = thetaDot0_;
            for (size_t j = 0; j < simSteps; ++j)
            {
                inputs_[0] = x_ / XDIVISOR;
                inputs_[1] = xDot_ / XDIVISOR;
                inputs_[2] = theta_ / TDIVISOR;
                inputs_[3] = thetaDot_ / TDIVISOR;
                if (std::abs(x_) > XDIVISOR || std::abs(xDot_) > XDIVISOR || std::abs(theta_) > TDIVISOR ||
                    std::abs(thetaDot_) > TDIVISOR)
                {
                    fitness = std::numeric_limits<T>::max();
                    return;
                }
                mNN->feedforward(inputs_, nn_out_, idx, false);

                params_copy.u = -nnKGain * (nn_out_[0] * (x_ - control::Params<T>().y_ref[0]) +
                                            nn_out_[1] * (xDot_ - control::Params<T>().y_ref[1]) +
                                            nn_out_[2] * (theta_ - control::Params<T>().y_ref[2]) +
                                            nn_out_[3] * (thetaDot_ - control::Params<T>().y_ref[3]));
                y1_           = {x_, xDot_, theta_, thetaDot_};
                ma::rk4singlestep(ge::motionFun, deltaTime, t_, y1_, y2_, params_copy);
                x_        = y2_[0];
                xDot_     = y2_[1];
                theta_    = y2_[2];
                thetaDot_ = y2_[3];
                // check if too big or too small
                if (x_ > T_C(1000) || x_ < -T_C(1000) || theta_ > T_C(1000) || theta_ < -T_C(1000))
                {
                    fitness = std::numeric_limits<T>::max();
                    return;
                }

                fitness += std::abs(x_ - control::Params<T>().y_ref[0]);
                fitness += std::abs(xDot_ - control::Params<T>().y_ref[1]);
                fitness += T_C(5) * std::abs(theta_ - control::Params<T>().y_ref[2]);
                fitness += std::abs(thetaDot_ - control::Params<T>().y_ref[3]);
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
    mNN->CreatePopulation(nn::NNParams().elitism);
    bestFitness = fitness[idx[0]];
    if (flags & pflags::VERBOSE)
    {
        for (size_t i = 0; i < nn::NNParams().top_individuals; ++i)
            LOGGER(logging::INFO) << std::string("Top " + num_to_string(i) + " : " + num_to_string(idx[i]) +
                                                 " fitness : " + num_to_string(fitness[idx[i]]));
        std::vector<T> inputs_(nn::NNParams().inputs), nn_out_(4);
        inputs_[0] = x0_ / XDIVISOR;
        inputs_[1] = xDot0_ / XDIVISOR;
        inputs_[2] = theta0_ / TDIVISOR;
        inputs_[3] = thetaDot0_ / TDIVISOR;
        mNN->feedforward(inputs_, nn_out_, 0, false);

        // print the best performer fitness
        LOGGER(logging::INFO) << std::string(
            "Best performer fitness : " + num_to_string(bestFitness) + " K: [" + num_to_string(K[0][0]) + " " +
            num_to_string(K[1][0]) + " " + num_to_string(K[2][0]) + " " + num_to_string(K[3][0]) + "]" + " K_NN: [" +
            num_to_string(nnKGain * nn_out_[0]) + " " + num_to_string(nnKGain * nn_out_[1]) + " " +
            num_to_string(nnKGain * nn_out_[2]) + " " + num_to_string(nnKGain * nn_out_[3]) + "]");
    }
    // save if necessary
    if (nn::NNParams().save_nn)
        if (!(nGenerations % nn::NNParams().save_every))
        {
            const std::string fname = nn::NNParams().save_path + nn::NNParams().name +
                                      (nn::NNParams().overwrite ? "" : "_" + std::to_string(nGenerations)) + ".hd5";
            // if file exists - delete it
            if (std::filesystem::exists(fname)) std::filesystem::remove(fname);
            mNN->Serialize(fname);
            if (flags & pflags::VERBOSE)
                LOGGER(logging::INFO) << std::string("Saved neural network to " + fname + " at generation " +
                                                     num_to_string(nGenerations));
        }
    nGenerations++;
    mNN->UpdateEpochs();
}

template <typename T> void ge::InvertedPendulum<T>::MultipleEpochNN()
{
    isTraining = true;
    for (size_t i = 0; i < nn::NNParams().epochsThread; ++i) TrainNN();
    isTraining = false;
}

template <typename T> void ge::InvertedPendulum<T>::LoadNN(const std::string& fname)
{
    std::string fname_;
    // if fname is empty - load from default path
    if (fname.empty())
    {
        std::cout << "Loading neural network from " << nn::NNParams().save_path << std::endl;
        // read all hd5 files in the directory and print them
        std::vector<std::string> files;
        for (const auto& entry : std::filesystem::directory_iterator(nn::NNParams().save_path))
            if (entry.path().extension() == ".hd5") files.push_back(entry.path().filename().string());

        // if no files found - return
        if (files.empty())
        {
            std::cout << "No files found in " << nn::NNParams().save_path << std::endl;
            return;
        }
        std::sort(files.begin(), files.end());
        for (size_t i = 0; i < files.size(); ++i) std::cout << i << " : " << files[i] << std::endl;
        std::cout << "Enter the number of the file to load : ";
        size_t idx;
        std::cin >> idx;
        if (idx >= files.size())
        {
            std::cout << "Invalid index" << std::endl;
            return;
        }
        fname_ = nn::NNParams().save_path + files[idx];
        std::cout << "Loading " << fname_ << std::endl;
    }
    else
    {
        if (!std::filesystem::exists(fname))
        {
            std::cout << "File " << fname << " does not exist" << std::endl;
            return;
        }
        fname_ = fname;
    }
    mNN = std::make_unique<nn::ANN_MLP_GA<T>>();
    mNN->SetName(nn::NNParams().name);
    mNN->Deserialize(fname_);
    nGenerations = mNN->GetEpochs() + 1;
    if (flags & pflags::VERBOSE) LOGGER(logging::INFO) << std::string("Loaded neural network from " + fname_);
}

template <typename T> void ge::InvertedPendulum<T>::ComputeMotion()
{
    std::array<T, 4>& (*fun_)(T, const std::array<T, 4>&, std::array<T, 4>&, const math::ode::Params<T>&) =
        ge::motionFun;
    std::array<T, 4> y1_ = {x, xDot, theta, thetaDot}, y2_;
    params.u             = 0;
    // use the neural network to compute the control signal
    if (flags & pflags::CONTROL_NN)
    {
        // compute the control signal
        std::vector<T> inputs_(nn::NNParams().inputs);
        inputs_[0] = x / XDIVISOR;
        inputs_[1] = xDot / XDIVISOR;
        inputs_[2] = theta / TDIVISOR;
        inputs_[3] = thetaDot / TDIVISOR;
        mNN->feedforward(inputs_, nnK, 0, false);
        params.u =
            -nnKGain *
            (nnK[0] * (x - control::Params<T>().y_ref[0]) + nnK[1] * (xDot - control::Params<T>().y_ref[1]) +
             nnK[2] * (theta - control::Params<T>().y_ref[2]) + nnK[3] * (thetaDot - control::Params<T>().y_ref[3]));
    }
    // use the LQR to compute the control signal
    else if (flags & pflags::CONTROL)
        for (size_t i = 0; i < 4; i++) params.u -= K[i][0] * (y1_[i] - control::Params<T>().y_ref[i]);
    ma::rk4singlestep(fun_, deltaTime, time, y1_, y2_, params);

    x        = y2_[0];
    xDot     = y2_[1];
    theta    = y2_[2];
    thetaDot = y2_[3];
    if (flags & pflags::VVERBOSE) PrintState();
}

template <typename T> void ge::InvertedPendulum<T>::SetFlag(int flag)
{
    flags |= flag;
}

template <typename T> void ge::InvertedPendulum<T>::UnsetFlag(int flag)
{
    flags &= ~flag;
}

template <typename T> void ge::InvertedPendulum<T>::ToggleFlag(int flag)
{
    flags ^= flag;
}

#undef XDIVISOR
#undef TDIVISOR
#undef T_C
#undef T_PI

// template instantiation
template class ge::InvertedPendulum<float>;
template class ge::InvertedPendulum<double>;
