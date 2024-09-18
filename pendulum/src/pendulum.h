#ifndef _PENDULUM_H_EDB04B0CA39E4B858C4E5689FA4C5563_
#define _PENDULUM_H_EDB04B0CA39E4B858C4E5689FA4C5563_

/************************/
/*      pendulum.h      */
/*    Version 1.0       */
/*     2023/04/13       */
/*  © Marco Azimonti    */
/************************/

#include <future>
#include <vector>
#include "algebra/matrix.h"
#include "ann_mlp_ga_v1.h"

namespace math
{
    namespace ode
    {

        template <typename T> struct Params
        {
            T omega2, gamma, u;
        };
    } // namespace ode
} // namespace math

namespace ge
{
#define T_C(x) static_cast<T>(x)

    namespace pflags
    {
        enum PendulumFlags : int {
            VERBOSE     = 1 << 0,
            VVERBOSE    = 1 << 1,
            GRID        = 1 << 2,
            F2D         = 1 << 3,
            CONTROL     = 1 << 4,
            NN_TRAIN    = 1 << 5,
            CONTROL_NN  = 1 << 6,
            USE_NN      = 1 << 7,
            ASYNC       = 1 << 8,
            MULTITHREAD = 1 << 9,
        };
    }

    template <typename T> class Pendulum
    {
      public:
        Pendulum();
        Pendulum(T length, T mass);
        ~Pendulum() = default;

        inline void SetLength(T t) { length = t; }

        inline void SetMass(T t) { mass = t; }

        inline void SetTheta(T t) { theta = t; }

        inline void SetThetaDot(T t) { thetaDot = t; }

        inline void SetThetaDDot(T t) { thetaDDot = t; }

        inline T GetLength() { return length; }

        inline T GetMass() { return mass; }

        inline T GetTheta() { return theta; }

        inline T GetThetaDot() { return thetaDot; }

        inline T GetThetaDDot() { return thetaDDot; }

        inline T GetTime() { return time; }

        inline T GetDeltaTime() { return deltaTime; }

        // GUI functions
        void onInit();
        void onUpdate();
        void PrintState();
        void PrintExtendedState();

        // Physics functions
        void ComputeMotion();
        void MultipleEpochNN();
        void TrainNN();
        void Reset();

        void SetFlag(int flag) { flags |= flag; }

        void UnsetFlag(int flag) { flags &= ~flag; }

        void ToggleFlag(int flag) { flags ^= flag; }

      private:
        void InitNN();
        std::vector<T>& ComputeControlNN(std::vector<T>& nn_out);
        std::future<void> mFuture;
        std::unique_ptr<nn::ANN_MLP_GA<T>> mNN{};
        T length, mass, theta, thetaDot, thetaDDot;
        T time, deltaTime;
        T bestFitness;
        math::ode::Params<T> params;
        int flags;
        size_t frames       = 0;
        size_t nGenerations = 0;
        bool isTraining     = false;
        la::Matrix<T> A{2, 2}, B{2, 1}, K{2, 1};
        T KGainNN;
    };

#undef T_C
} // namespace ge
#endif
