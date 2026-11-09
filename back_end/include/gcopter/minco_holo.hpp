/*
    Dimension-generic minimum-jerk MINCO (s = 3, non-uniform time), used by the planar holonomic ROV planner.
    The flat output is the pose itself, sigma(t) = (x, y, theta), so Dim = 3.

    This is the same construction as minco::MINCO_S3NU in minco.hpp (GCOPTER, MIT License,
    Copyright (c) 2021 Zhepei Wang), with the hard-coded 2 columns replaced by a template dimension and only the
    functions the holonomic planner needs (no arc-length relaxation: the terminal pose is a hard boundary condition).
*/

#ifndef MINCO_HOLO_HPP
#define MINCO_HOLO_HPP

#include "gcopter/minco.hpp"
#include "gcopter/trajectory.hpp"

#include <Eigen/Eigen>

namespace minco
{
    template <int Dim>
    class MINCO_S3NU_ND
    {
    public:
        typedef Eigen::Matrix<double, Eigen::Dynamic, Dim> CoeffMat;    // (6N) x Dim, low order first per piece
        typedef Eigen::Matrix<double, Dim, 3> PVA;                     // columns: position, velocity, acceleration
        typedef Eigen::Matrix<double, Dim, Eigen::Dynamic> PointMat;

        MINCO_S3NU_ND() = default;
        ~MINCO_S3NU_ND() { A.destroy(); }

    private:
        int N = 0;
        PVA headPVA, tailPVA;
        BandedSystem A;
        CoeffMat b;
        Eigen::VectorXd T1, T2, T3, T4, T5;
        Eigen::Matrix<double, Dim, 1> w;     // per-dimension jerk-energy weights

    public:
        inline void setConditions(const PVA &headState, const PVA &tailState, const int &pieceNum,
                                  const Eigen::Matrix<double, Dim, 1> &energyWt)
        {
            if (N > 0) A.destroy();
            N = pieceNum;
            headPVA = headState;
            tailPVA = tailState;
            A.create(6 * N, 6, 6);
            b.resize(6 * N, Dim);
            T1.resize(N); T2.resize(N); T3.resize(N); T4.resize(N); T5.resize(N);
            w = energyWt;
        }

        inline void setHConditions(const PVA &headState) { headPVA = headState; }
        inline void setTConditions(const PVA &tailState) { tailPVA = tailState; }

        inline void setParameters(const PointMat &inPs, const Eigen::VectorXd &ts)
        {
            T1 = ts;
            T2 = T1.cwiseProduct(T1);
            T3 = T2.cwiseProduct(T1);
            T4 = T2.cwiseProduct(T2);
            T5 = T4.cwiseProduct(T1);

            A.reset();
            b.setZero();

            A(0, 0) = 1.0;
            A(1, 1) = 1.0;
            A(2, 2) = 2.0;
            b.row(0) = headPVA.col(0).transpose();
            b.row(1) = headPVA.col(1).transpose();
            b.row(2) = headPVA.col(2).transpose();

            for (int i = 0; i < N - 1; i++)
            {
                A(6 * i + 3, 6 * i + 3) = 6.0;
                A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                A(6 * i + 3, 6 * i + 9) = -6.0;
                A(6 * i + 4, 6 * i + 4) = 24.0;
                A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                A(6 * i + 4, 6 * i + 10) = -24.0;
                A(6 * i + 5, 6 * i) = 1.0;
                A(6 * i + 5, 6 * i + 1) = T1(i);
                A(6 * i + 5, 6 * i + 2) = T2(i);
                A(6 * i + 5, 6 * i + 3) = T3(i);
                A(6 * i + 5, 6 * i + 4) = T4(i);
                A(6 * i + 5, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i) = 1.0;
                A(6 * i + 6, 6 * i + 1) = T1(i);
                A(6 * i + 6, 6 * i + 2) = T2(i);
                A(6 * i + 6, 6 * i + 3) = T3(i);
                A(6 * i + 6, 6 * i + 4) = T4(i);
                A(6 * i + 6, 6 * i + 5) = T5(i);
                A(6 * i + 6, 6 * i + 6) = -1.0;
                A(6 * i + 7, 6 * i + 1) = 1.0;
                A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                A(6 * i + 7, 6 * i + 7) = -1.0;
                A(6 * i + 8, 6 * i + 2) = 2.0;
                A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                A(6 * i + 8, 6 * i + 8) = -2.0;

                b.row(6 * i + 5) = inPs.col(i).transpose();
            }

            A(6 * N - 3, 6 * N - 6) = 1.0;
            A(6 * N - 3, 6 * N - 5) = T1(N - 1);
            A(6 * N - 3, 6 * N - 4) = T2(N - 1);
            A(6 * N - 3, 6 * N - 3) = T3(N - 1);
            A(6 * N - 3, 6 * N - 2) = T4(N - 1);
            A(6 * N - 3, 6 * N - 1) = T5(N - 1);
            A(6 * N - 2, 6 * N - 5) = 1.0;
            A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
            A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
            A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
            A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
            A(6 * N - 1, 6 * N - 4) = 2;
            A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
            A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
            A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

            b.row(6 * N - 3) = tailPVA.col(0).transpose();
            b.row(6 * N - 2) = tailPVA.col(1).transpose();
            b.row(6 * N - 1) = tailPVA.col(2).transpose();

            A.factorizeLU();
            A.solve(b);
        }

        inline void getTrajectory(Trajectory<5, Dim> &traj) const
        {
            traj.clear();
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i), b.template block<6, Dim>(6 * i, 0).transpose().rowwise().reverse());
            }
        }

        inline const CoeffMat &getCoeffs(void) const { return b; }

        // Weighted jerk energy sum_i int_0^Ti  j(t)^T W j(t) dt
        inline void getEnergy(double &energy) const
        {
            energy = 0.0;
            for (int i = 0; i < N; i++)
            {
                const auto c3 = b.row(6 * i + 3), c4 = b.row(6 * i + 4), c5 = b.row(6 * i + 5);
                energy += 36.0 * c3.cwiseProduct(w.transpose()).dot(c3) * T1(i) +
                          144.0 * c4.cwiseProduct(w.transpose()).dot(c3) * T2(i) +
                          192.0 * c4.cwiseProduct(w.transpose()).dot(c4) * T3(i) +
                          240.0 * c5.cwiseProduct(w.transpose()).dot(c3) * T3(i) +
                          720.0 * c5.cwiseProduct(w.transpose()).dot(c4) * T4(i) +
                          720.0 * c5.cwiseProduct(w.transpose()).dot(c5) * T5(i);
            }
        }

        inline void getEnergyPartialGradByCoeffs(CoeffMat &gdC) const
        {
            gdC.resize(6 * N, Dim);
            for (int i = 0; i < N; i++)
            {
                const Eigen::Matrix<double, 1, Dim> c3 = b.row(6 * i + 3).cwiseProduct(w.transpose());
                const Eigen::Matrix<double, 1, Dim> c4 = b.row(6 * i + 4).cwiseProduct(w.transpose());
                const Eigen::Matrix<double, 1, Dim> c5 = b.row(6 * i + 5).cwiseProduct(w.transpose());
                gdC.row(6 * i + 5) = 240.0 * c3 * T3(i) + 720.0 * c4 * T4(i) + 1440.0 * c5 * T5(i);
                gdC.row(6 * i + 4) = 144.0 * c3 * T2(i) + 384.0 * c4 * T3(i) + 720.0 * c5 * T4(i);
                gdC.row(6 * i + 3) = 72.0 * c3 * T1(i) + 144.0 * c4 * T2(i) + 240.0 * c5 * T3(i);
                gdC.template block<3, Dim>(6 * i, 0).setZero();
            }
        }

        inline void getEnergyPartialGradByTimes(Eigen::VectorXd &gdT) const
        {
            gdT.resize(N);
            for (int i = 0; i < N; i++)
            {
                const auto c3 = b.row(6 * i + 3), c4 = b.row(6 * i + 4), c5 = b.row(6 * i + 5);
                gdT(i) = 36.0 * c3.cwiseProduct(w.transpose()).dot(c3) +
                         288.0 * c4.cwiseProduct(w.transpose()).dot(c3) * T1(i) +
                         576.0 * c4.cwiseProduct(w.transpose()).dot(c4) * T2(i) +
                         720.0 * c5.cwiseProduct(w.transpose()).dot(c3) * T2(i) +
                         2880.0 * c5.cwiseProduct(w.transpose()).dot(c4) * T3(i) +
                         3600.0 * c5.cwiseProduct(w.transpose()).dot(c5) * T4(i);
            }
        }

        // Chain rule from (coefficients, times) to (inner points, times); the boundary states are fixed.
        inline void propogateGrad(const CoeffMat &partialGradByCoeffs, const Eigen::VectorXd &partialGradByTimes,
                                  PointMat &gradByPoints, Eigen::VectorXd &gradByTimes)
        {
            gradByPoints.resize(Dim, N - 1);
            gradByTimes.resize(N);
            CoeffMat adjGrad = partialGradByCoeffs;
            A.solveAdj(adjGrad);

            for (int i = 0; i < N - 1; i++)
                gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();

            Eigen::Matrix<double, 6, Dim> B1;
            Eigen::Matrix<double, 3, Dim> B2;
            for (int i = 0; i < N - 1; i++)
            {
                B1.row(2) = -(b.row(i * 6 + 1) + 2.0 * T1(i) * b.row(i * 6 + 2) + 3.0 * T2(i) * b.row(i * 6 + 3) +
                              4.0 * T3(i) * b.row(i * 6 + 4) + 5.0 * T4(i) * b.row(i * 6 + 5));
                B1.row(3) = B1.row(2);
                B1.row(4) = -(2.0 * b.row(i * 6 + 2) + 6.0 * T1(i) * b.row(i * 6 + 3) +
                              12.0 * T2(i) * b.row(i * 6 + 4) + 20.0 * T3(i) * b.row(i * 6 + 5));
                B1.row(5) = -(6.0 * b.row(i * 6 + 3) + 24.0 * T1(i) * b.row(i * 6 + 4) + 60.0 * T2(i) * b.row(i * 6 + 5));
                B1.row(0) = -(24.0 * b.row(i * 6 + 4) + 120.0 * T1(i) * b.row(i * 6 + 5));
                B1.row(1) = -120.0 * b.row(i * 6 + 5);
                gradByTimes(i) = B1.cwiseProduct(adjGrad.template block<6, Dim>(6 * i + 3, 0)).sum();
            }
            B2.row(0) = -(b.row(6 * N - 5) + 2.0 * T1(N - 1) * b.row(6 * N - 4) + 3.0 * T2(N - 1) * b.row(6 * N - 3) +
                          4.0 * T3(N - 1) * b.row(6 * N - 2) + 5.0 * T4(N - 1) * b.row(6 * N - 1));
            B2.row(1) = -(2.0 * b.row(6 * N - 4) + 6.0 * T1(N - 1) * b.row(6 * N - 3) +
                          12.0 * T2(N - 1) * b.row(6 * N - 2) + 20.0 * T3(N - 1) * b.row(6 * N - 1));
            B2.row(2) = -(6.0 * b.row(6 * N - 3) + 24.0 * T1(N - 1) * b.row(6 * N - 2) + 60.0 * T2(N - 1) * b.row(6 * N - 1));
            gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.template block<3, Dim>(6 * N - 3, 0)).sum();
            gradByTimes += partialGradByTimes;
        }
    };
}

#endif
