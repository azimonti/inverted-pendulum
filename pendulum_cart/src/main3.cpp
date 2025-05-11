/************************/
/*     main3.cpp        */
/*    Version 2.0       */
/*     2025/05/11       */
/************************/

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <signal.h>
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
} // namespace ge2

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
    // HDF5 writer removed
    (void)bVerbose;
    (void)bVVerbose;

    // Set precision for output files
    const int output_precision = 8;

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

    // Write closeloop simulation data to text file
    std::ofstream closeloop_file;
    std::string closeloop_filename = "./externals/ma-libs/build/" + sName + "_closeloop.txt";
    closeloop_file.open(closeloop_filename);
    if (closeloop_file.is_open())
    {
        closeloop_file << std::fixed << std::setprecision(output_precision);

        // Write K gain
        closeloop_file << "K_Gain: " << mInvPendulum.GetKGain() << std::endl;

        // Write K values
        closeloop_file << "K_Values: ";
        const auto& k_values_cl = mInvPendulum.GetK();
        for (size_t k_idx = 0; k_idx < k_values_cl.size(); ++k_idx)
        {
            closeloop_file << k_values_cl[k_idx] << (k_idx == k_values_cl.size() - 1 ? "" : ", ");
        }
        closeloop_file << std::endl;

        // Write data headers
        closeloop_file << "time,x,x_dot,theta,theta_dot" << std::endl;
        // Write data
        for (size_t i_data = 0; i_data < simSteps; ++i_data)
        {
            closeloop_file << time[i_data] << "," << vX[i_data] << "," << vXDot[i_data] << "," << vTheta[i_data] << ","
                           << vThetaDot[i_data] << std::endl;
        }
        closeloop_file.close();
        LOGGER(logging::INFO) << "Closed-loop simulation data written to " << closeloop_filename;
    }
    else { LOGGER(logging::ERROR) << "Failed to open file for writing: " << closeloop_filename; }

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

        // Write NN simulation data to text file
        std::ofstream nn_sim_file;
        std::string nn_sim_filename = "./externals/ma-libs/build/" + sName + "_nn_simul_" + std::to_string(i) + ".txt";
        nn_sim_file.open(nn_sim_filename);
        if (nn_sim_file.is_open())
        {
            nn_sim_file << std::fixed << std::setprecision(output_precision);

            nn_sim_file << "Fitness: " << bestFitness << std::endl;

            // Write NN K gain
            nn_sim_file << "NN_K_Gain: " << mInvPendulum.GetNnKGain() << std::endl;

            // Write data headers
            nn_sim_file << "time,x,x_dot,theta,theta_dot,nnk_c1,nnk_c2,nnk_c3,nnk_c4" << std::endl;
            // Write data
            for (size_t i_data = 0; i_data < simSteps; ++i_data)
            {
                nn_sim_file << time[i_data] << "," << vX[i_data] << "," << vXDot[i_data] << "," << vTheta[i_data] << ","
                            << vThetaDot[i_data];
                for (size_t k_comp = 0; k_comp < vKNN[i_data].size(); ++k_comp)
                {
                    nn_sim_file << "," << vKNN[i_data][k_comp];
                }
                nn_sim_file << std::endl;
            }
            nn_sim_file.close();
            LOGGER(logging::INFO) << "NN simulation data for iteration " << i << " written to " << nn_sim_filename;
        }
        else { LOGGER(logging::ERROR) << "Failed to open file for writing: " << nn_sim_filename; }

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
