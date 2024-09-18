/************************/
/*     main3.cpp        */
/*    Version 1.0       */
/*     2023/05/12       */
/************************/

#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <vector>
#include <signal.h>
#include "hdf5/hdf5_ext.h"
#include "log/log.h"
#include "pendulum_cart.h"

#define F_PI static_cast<float>(3.14159265358979323846)

namespace ge2
{
    template <typename T> std::string num_to_string(T num, int decimals = 2)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(decimals) << num;
        return ss.str();
    }
} // namespace ge

// handle SIGNINT
bool bSignal = false;
std::string sSignal;

void handle_sigint(int sig)
{
    (void)sig;
    std::cout << "INPUT H FOR HELP OR SELECT THE ACTION: ";
    std::getline(std::cin, sSignal);
    if (sSignal[0] == 'h' || sSignal[0] == 'H')
    {
        std::vector<std::string> choices           = {"Q", "V", "W"};
        std::vector<std::string> choices_desc      = {"QUIT", "VERBOSE", "VVERBOSE"};
        std::vector<std::string> choices_desc_long = {"TOGGLE CONTROL RESPONSE", "LOAD NN", "QUIT THE PROGRAM",
                                                      "TOGGLE VERBOSE", "TOGGLE VERY VERBOSE"};
        std::cout << "***************************************" << std::endl;
        std::cout << "CHOOSE ONE OF THE FOLLOWING OPTIONS:" << std::endl;
        for (size_t i = 0; i < choices.size(); i++)
            std::cout << "* " << choices[i] << " - " << choices_desc[i] << " - " << choices_desc_long[i] << std::endl;
        std::cout << "***************************************" << std::endl;
        return;
    }
    bSignal = true;
}

int main()
{
    bool bFileLog = false;
    LOGGER_PARAM(logging::LEVELMAX, logging::INFO);
    LOGGER_PARAM(logging::LOGTIME, true);
    if (bFileLog)
    {
        LOGGER_PARAM(logging::FILENAME, "out_invpendulum.log");
        LOGGER_PARAM(logging::FILEOUT, true);
    }
#ifndef WIN32
    signal(SIGINT, handle_sigint);
#endif
    ge::InvertedPendulum<float> mInvPendulum;
    float bestFitness;
    bool bVerbose = false, bVVerbose = false;
    const float deltaTime = mInvPendulum.GetDeltaTime();
    const size_t simSteps = static_cast<size_t>(240.0f / deltaTime);
    std::vector<float> time(simSteps, 0.0f), vX(simSteps, 0.0f), vXDot(simSteps, 0.0f), vTheta(simSteps, 0.0f),
        vThetaDot(simSteps, 0.0f);
    std::vector<std::vector<float>> vKNN(simSteps, std::vector<float>(4, 0.0f));
#ifdef UP_POSITION
    const std::string sName = "up_simul";
#else
    const std::string sName = "down_simul";
#endif
    h5::H5ppWriter h5("./build/archive/" + sName + ".h5");
    (void)bVerbose;
    (void)bVVerbose;
    mInvPendulum.onInit();
    // archive the close-loop simulation
    mInvPendulum.SetFlag(ge::pflags::CONTROL);
    mInvPendulum.Reset();
    for (size_t i = 0; i < simSteps; i++)
    {
        mInvPendulum.ComputeMotion();
        mInvPendulum.IncrementTime();
        time[i]      = mInvPendulum.GetTime();
        vX[i]        = mInvPendulum.GetX();
        vXDot[i]     = mInvPendulum.GetXDot();
        vTheta[i]    = mInvPendulum.GetTheta();
        // ensure that theta is in the range [0, 2* pi], adding a small offset to avoid numerical errors
        vTheta[i]    = vTheta[i] - 2.0f * F_PI * std::floor((std::abs(vTheta[i]) + 0.0001f) / (2.0f * F_PI));
        vThetaDot[i] = mInvPendulum.GetThetaDot();
    }
    h5.write("simulation/closeloop/time", time);
    h5.write("simulation/closeloop/x", vX);
    h5.write("simulation/closeloop/xdot", vXDot);
    h5.write("simulation/closeloop/theta", vTheta);
    h5.write("simulation/closeloop/thetadot", vThetaDot);
    h5.write("simulation/closeloop/kgain", mInvPendulum.GetKGain());
    h5.write("simulation/closeloop/k", mInvPendulum.GetK());

    // archive the NN simulation
    mInvPendulum.SetFlag(ge::pflags::CONTROL_NN);
    for (int i = 250; i <= 10000; i += 250)
    {
        mInvPendulum.LoadNN("../../_var/assets/nn/SIMULATIONS/inverted_pendulum/" + sName + "/invpendulum_" +
                            std::to_string(i) + ".hd5");
        std::cout << "LOADED FILE: " << i << std::endl;
        bestFitness = mInvPendulum.ComputeBestFitness();
        // compute the motion
        mInvPendulum.Reset();
        for (size_t i = 0; i < simSteps; i++)
        {
            mInvPendulum.ComputeMotion();
            mInvPendulum.IncrementTime();
            time[i]      = mInvPendulum.GetTime();
            vX[i]        = mInvPendulum.GetX();
            vXDot[i]     = mInvPendulum.GetXDot();
            vTheta[i]    = mInvPendulum.GetTheta();
            // ensure that theta is in the range [0, 2* pi], adding a small offset to avoid numerical errors
            vTheta[i]    = vTheta[i] - 2.0f * F_PI * std::floor((std::abs(vTheta[i]) + 0.0001f) / (2.0f * F_PI));
            vThetaDot[i] = mInvPendulum.GetThetaDot();
            vKNN[i]      = mInvPendulum.GetNnK();
        }

        h5.write("simulation/" + std::to_string(i) + "/time", time);
        h5.write("simulation/" + std::to_string(i) + "/fitness", bestFitness);
        h5.write("simulation/" + std::to_string(i) + "/x", vX);
        h5.write("simulation/" + std::to_string(i) + "/xdot", vXDot);
        h5.write("simulation/" + std::to_string(i) + "/theta", vTheta);
        h5.write("simulation/" + std::to_string(i) + "/thetadot", vThetaDot);
        h5.write("simulation/" + std::to_string(i) + "/nnkgain", mInvPendulum.GetNnKGain());
        h5.write("simulation/" + std::to_string(i) + "/nnk", vKNN);

        std::cout << "best fitness: " << bestFitness << " time: " << ge2::num_to_string(time.back())
                  << " x: " << ge2::num_to_string(vX.back()) << " xdot: " << ge2::num_to_string(vXDot.back())
                  << " theta: " << ge2::num_to_string(vTheta.back())
                  << " thetadot: " << ge2::num_to_string(vThetaDot.back()) << std::endl;

        if (bSignal)
        {
            switch (sSignal[0])
            {
            case 'q':
            case 'Q':
                LOGGER(logging::INFO) << std::string("QUIT");
                exit(0);
                break;
            case 'v':
            case 'V':
                LOGGER(logging::INFO) << std::string("TOGGLE VERBOSE");
                bVerbose ^= 1;
                break;
            case 'w':
            case 'W':
                LOGGER(logging::INFO) << std::string("TOGGLE VERY VERBOSE");
                bVVerbose ^= 1;
                break;
            }
            bSignal = false;
        }
    }
    return 0;
}

#undef F_PI
