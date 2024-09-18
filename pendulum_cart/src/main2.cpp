/************************/
/*     main2.cpp        */
/*    Version 1.0       */
/*     2023/04/24       */
/************************/

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <signal.h>
#include "log/log.h"
#include "pendulum_cart.h"

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
        std::vector<std::string> choices           = {"C", "L", "M", "N", "Q", "T", "V", "W"};
        std::vector<std::string> choices_desc      = {"CONTROL", "LOAD",  "MULTITHREAD", "NEURAL_NETWORK",
                                                      "QUIT",    "TRAIN", "VERBOSE",     "VVERBOSE"};
        std::vector<std::string> choices_desc_long = {"TOGGLE CONTROL RESPONSE", "LOAD NN",
                                                      "TOGGLE MULTITHREAD",      "USE NEURAL NETWORK",
                                                      "QUIT THE PROGRAM",        "TRAIN THE NEURAL NETWORK",
                                                      "TOGGLE VERBOSE",          "TOGGLE VERY VERBOSE"};
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
    mInvPendulum.onInit();
    // set time counter
    auto t1 = std::chrono::high_resolution_clock::now(), ts = t1;
    int SIM_MAX_SECONDS = 3, SIM_MS = 5000;
    bool bFixTime = false, bPrintState = true;
    bool bVerbose = false, bVVerbose = false;
    (void)bVerbose;
    (void)bVVerbose;
    mInvPendulum.SetFlag(ge::pflags::NN_TRAIN);
    mInvPendulum.ToggleFlag(ge::pflags::MULTITHREAD);
    mInvPendulum.LoadNN();

    while (true)
    {
        // print the state every 5 seconds without sleep
        auto t2 = std::chrono::high_resolution_clock::now();
        if (bPrintState && std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() >= SIM_MS)
        {
            mInvPendulum.ComputeMotion();
            mInvPendulum.IncrementTime();
            mInvPendulum.PrintExtendedState();
            t1 = std::chrono::high_resolution_clock::now();
        }
        if (bFixTime && std::chrono::duration_cast<std::chrono::seconds>(t1 - ts).count() >= SIM_MAX_SECONDS) exit(0);
        mInvPendulum.TrainNN();
        if (bSignal)
        {
            switch (sSignal[0])
            {
            case 'c':
            case 'C':
                LOGGER(logging::INFO) << std::string("TOGGLE CONTROL RESPONSE");
                mInvPendulum.ToggleFlag(ge::pflags::CONTROL);
                break;
            case 'l':
            case 'L':
                LOGGER(logging::INFO) << std::string("LOAD NN");
                mInvPendulum.LoadNN();
                break;
            case 'm':
            case 'M':
                LOGGER(logging::INFO) << std::string("TOGGLE MULTITHREAD");
                mInvPendulum.ToggleFlag(ge::pflags::MULTITHREAD);
                break;
            case 'n':
            case 'N':
                LOGGER(logging::INFO) << std::string("TOGGLE NEURAL NETWORK");
                mInvPendulum.ToggleFlag(ge::pflags::CONTROL_NN);
                break;
            case 'q':
            case 'Q':
                LOGGER(logging::INFO) << std::string("QUIT");
                exit(0);
                break;
            case 't':
            case 'T':
                LOGGER(logging::INFO) << std::string("TOGGLE TRAIN");
                mInvPendulum.ToggleFlag(ge::pflags::NN_TRAIN);
                break;
            case 'v':
            case 'V':
                LOGGER(logging::INFO) << std::string("TOGGLE VERBOSE");
                mInvPendulum.ToggleFlag(ge::pflags::VERBOSE);
                bVerbose ^= 1;
                break;
            case 'w':
            case 'W':
                LOGGER(logging::INFO) << std::string("TOGGLE VERY VERBOSE");
                mInvPendulum.ToggleFlag(ge::pflags::VVERBOSE);
                bVVerbose ^= 1;
                break;
            }
            bSignal = false;
        }
    }
    return 0;
}
