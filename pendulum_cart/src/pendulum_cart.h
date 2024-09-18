#ifndef _PENDULUM_CART_H_EDB04B0CA39E4B858C4E5689FA4C5563_
#define _PENDULUM_CART_H_EDB04B0CA39E4B858C4E5689FA4C5563_

/************************/
/*   pendulum_cart.h    */
/*    Version 1.0       */
/*     2023/04/13       */
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
            T m, M, L, g, b, u;
        };
    } // namespace ode
} // namespace math

namespace ge
{
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

    template <typename T> class InvertedPendulum
    {
      public:
        InvertedPendulum();
        InvertedPendulum(T length, T mass1, T mass2);
        ~InvertedPendulum() = default;

        inline void SetLength(T t) { length = t; }

        inline void SetMass1(T t) { mass1 = t; }

        inline void SetTheta(T t) { theta = t; }

        inline void SetThetaDot(T t) { thetaDot = t; }

        inline void SetMass2(T t) { mass2 = t; }

        inline void SetX(T t) { x = t; }

        inline void SetXDot(T t) { xDot = t; }

        inline void SetDeltaTime(T t) { deltaTime = t; }

        inline T GetLength() { return length; }

        inline T GetMass1() { return mass1; }

        inline T GetTheta() { return theta; }

        inline T GetThetaDot() { return thetaDot; }

        inline T GetMass2() { return mass2; }

        inline T GetX() { return x; }

        inline T GetXDot() { return xDot; }

        inline T GetTime() { return time; }

        inline T GetDeltaTime() { return deltaTime; }

        inline T GetKGain() { return kGain; }

        inline const std::vector<T>& GetK() { return K.data(); }

        inline T GetBestFitness() { return bestFitness; }

        inline T GetNnKGain() { return nnKGain; }

        inline const std::vector<T>& GetNnK() { return nnK; }

        // GUI functions
        void onInit();
        void onUpdate();
        void PrintState();
        void PrintExtendedState();

        // Simulation functions
        void Reset();

        inline void IncrementTime() { time += deltaTime; }

        void ComputeMotion();

        // NN functions
        T ComputeBestFitness();
        void LoadNN(const std::string& fname = "");
        void MultipleEpochNN();
        void TrainNN();

        void SetFlag(int flag);
        void UnsetFlag(int flag);
        void ToggleFlag(int flag);

      private:
        void InitNN();
        std::future<void> mFuture;
        std::unique_ptr<nn::ANN_MLP_GA<T>> mNN{};
        T length, mass1, theta, thetaDot;
        T mass2, x, xDot;
        T time, deltaTime;
        T bestFitness;
        math::ode::Params<T> params;
        int flags;
        size_t frames       = 0;
        size_t nGenerations = 0;
        bool isTraining     = false;
        la::Matrix<T> A{4, 4}, B{4, 1}, K{4, 1};
        T kGain;
        std::vector<T> nnK;
        T nnKGain;
    };
} // namespace ge
#endif
