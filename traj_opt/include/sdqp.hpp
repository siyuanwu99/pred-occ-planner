/*
    MIT License

    Copyright (c) 2022 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SDQP_HPP
#define SDQP_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <random>

namespace sdqp
{
    constexpr double eps = 1.0e-12;

    enum
    {
        MINIMUM = 0,
        INFEASIBLE,
    };

    template <int d>
    inline void set_zero(double *x)
    {
        for (int i = 0; i < d; ++i)
        {
            x[i] = 0.0;
        }
        return;
    }

    template <int d>
    inline double dot(const double *x,
                      const double *y)
    {
        double s = 0.0;
        for (int i = 0; i < d; ++i)
        {
            s += x[i] * y[i];
        }
        return s;
    }

    template <int d>
    inline double sqr_norm(const double *x)
    {
        double s = 0.0;
        for (int i = 0; i < d; ++i)
        {
            s += x[i] * x[i];
        }
        return s;
    }

    template <int d>
    inline void mul(const double *x,
                    const double s,
                    double *y)
    {
        for (int i = 0; i < d; ++i)
        {
            y[i] = x[i] * s;
        }
        return;
    }

    template <int d>
    inline int max_abs(const double *x)
    {
        int id = 0;
        double mag = std::fabs(x[0]);
        for (int i = 1; i < d; ++i)
        {
            const double s = std::fabs(x[i]);
            if (s > mag)
            {
                id = i;
                mag = s;
            }
        }
        return id;
    }

    template <int d>
    inline void cpy(const double *x,
                    double *y)
    {
        for (int i = 0; i < d; ++i)
        {
            y[i] = x[i];
        }
        return;
    }

    inline int move_to_front(const int i,
                             int *next,
                             int *prev)
    {
        if (i == 0 || i == next[0])
        {
            return i;
        }
        const int previ = prev[i];
        next[prev[i]] = next[i];
        prev[next[i]] = prev[i];
        next[i] = next[0];
        prev[i] = 0;
        prev[next[i]] = i;
        next[0] = i;
        return previ;
    }

    template <int d>
    inline int min_norm(const double *halves,
                        const int n,
                        const int m,
                        double *opt,
                        double *work,
                        int *next,
                        int *prev)
    {
        int status = MINIMUM;
        set_zero<d>(opt);
        if (m <= 0)
        {
            return status;
        }

        double *reflx = work;
        double *new_opt = reflx + d;
        double *new_halves = new_opt + (d - 1);
        double *new_work = new_halves + n * d;

        for (int i = 0; i != m; i = next[i])
        {
            const double *plane_i = halves + (d + 1) * i;

            if (dot<d>(opt, plane_i) + plane_i[d] > (d + 1) * eps)
            {
                const double s = sqr_norm<d>(plane_i);

                if (s < (d + 1) * eps * eps)
                {
                    return INFEASIBLE;
                }

                mul<d>(plane_i, -plane_i[d] / s, opt);

                if (i == 0)
                {
                    continue;
                }

                // stable Householder reflection with pivoting
                const int id = max_abs<d>(opt);
                const double xnorm = std::sqrt(sqr_norm<d>(opt));
                cpy<d>(opt, reflx);
                reflx[id] += opt[id] < 0.0 ? -xnorm : xnorm;
                const double h = -2.0 / sqr_norm<d>(reflx);

                for (int j = 0; j != i; j = next[j])
                {
                    double *new_plane = new_halves + d * j;
                    const double *old_plane = halves + (d + 1) * j;
                    const double coeff = h * dot<d>(old_plane, reflx);
                    for (int k = 0; k < d; ++k)
                    {
                        const int l = k < id ? k : k - 1;
                        new_plane[l] = k != id ? old_plane[k] + reflx[k] * coeff : new_plane[l];
                    }
                    new_plane[d - 1] = dot<d>(opt, old_plane) + old_plane[d];
                }

                status = min_norm<d - 1>(new_halves, n, i, new_opt, new_work, next, prev);

                if (status == INFEASIBLE)
                {
                    return INFEASIBLE;
                }

                double coeff = 0.0;
                for (int j = 0; j < d; ++j)
                {
                    const int k = j < id ? j : j - 1;
                    coeff += j != id ? reflx[j] * new_opt[k] : 0.0;
                }
                coeff *= h;
                for (int j = 0; j < d; ++j)
                {
                    const int k = j < id ? j : j - 1;
                    opt[j] += j != id ? new_opt[k] + reflx[j] * coeff : reflx[j] * coeff;
                }

                i = move_to_front(i, next, prev);
            }
        }

        return status;
    }

    template <>
    inline int min_norm<1>(const double *halves,
                           const int n,
                           const int m,
                           double *opt,
                           double *work,
                           int *next,
                           int *prev)
    {
        opt[0] = 0.0;
        bool l = false;
        bool r = false;

        for (int i = 0; i != m; i = next[i])
        {
            const double a = halves[2 * i];
            const double b = halves[2 * i + 1];
            if (a * opt[0] + b > 2.0 * eps)
            {
                if (std::fabs(a) < 2.0 * eps)
                {
                    return INFEASIBLE;
                }

                l = l || a < 0.0;
                r = r || a > 0.0;

                if (l && r)
                {
                    return INFEASIBLE;
                }

                opt[0] = -b / a;
            }
        }

        return MINIMUM;
    }

    inline void rand_permutation(const int n,
                                 int *p)
    {
        typedef std::uniform_int_distribution<int> rand_int;
        typedef rand_int::param_type rand_range;
        static std::mt19937_64 gen;
        static rand_int rdi(0, 1);
        int j, k;
        for (int i = 0; i < n; ++i)
        {
            p[i] = i;
        }
        for (int i = 0; i < n; ++i)
        {
            rdi.param(rand_range(0, n - i - 1));
            j = rdi(gen) + i;
            k = p[j];
            p[j] = p[i];
            p[i] = k;
        }
    }

    template <int d>
    inline double sdmn(const Eigen::Matrix<double, -1, d> &A,
                       const Eigen::Matrix<double, -1, 1> &b,
                       Eigen::Matrix<double, d, 1> &x)
    {
        x.setZero();
        const int n = b.size();
        if (n < 1)
        {
            return 0.0;
        }

        Eigen::VectorXi perm(n - 1);
        Eigen::VectorXi next(n);
        Eigen::VectorXi prev(n + 1);
        if (n > 1)
        {
            rand_permutation(n - 1, perm.data());
            prev(0) = 0;
            next(0) = perm(0) + 1;
            prev(perm(0) + 1) = 0;
            for (int i = 0; i < n - 2; ++i)
            {
                next(perm(i) + 1) = perm(i + 1) + 1;
                prev(perm(i + 1) + 1) = perm(i) + 1;
            }
            next(perm(n - 2) + 1) = n;
        }
        else
        {
            prev(0) = 0;
            next(0) = 1;
            next(1) = 1;
        }

        Eigen::Matrix<double, d + 1, -1, Eigen::ColMajor> halves(d + 1, n);
        Eigen::VectorXd work((n + 2) * (d + 2) * (d - 1) / 2 + 1 - d);

        const Eigen::VectorXd scale = A.rowwise().norm();
        halves.template topRows<d>() = (A.array().colwise() / scale.array()).transpose();
        halves.template bottomRows<1>() = (-b.array() / scale.array()).transpose();

        const int status = min_norm<d>(halves.data(), n, n,
                                       x.data(), work.data(),
                                       next.data(), prev.data());

        double minimum = INFINITY;
        if (status != INFEASIBLE)
        {
            minimum = x.norm();
        }

        return minimum;
    }

    /**
     * minimize     0.5 x' Q x + c' x
     * subject to       A x <= b
     * Q must be positive definite
     **/

    template <int d>
    inline double sdqp(const Eigen::Matrix<double, d, d> &Q,
                       const Eigen::Matrix<double, d, 1> &c,
                       const Eigen::Matrix<double, -1, d> &A,
                       const Eigen::Matrix<double, -1, 1> &b,
                       Eigen::Matrix<double, d, 1> &x)
    {
        Eigen::LLT<Eigen::Matrix<double, d, d>> llt(Q);
        if (llt.info() != Eigen::Success)
        {
            return INFINITY;
        }

        const Eigen::Matrix<double, -1, d> As = llt.matrixU().template solve<Eigen::OnTheRight>(A);
        const Eigen::Matrix<double, d, 1> v = llt.solve(c);
        const Eigen::Matrix<double, -1, 1> bs = A * v + b;

        double minimum = sdmn<d>(As, bs, x);
        if (!std::isinf(minimum))
        {
            llt.matrixU().template solveInPlace<Eigen::OnTheLeft>(x);
            x -= v;
            minimum = 0.5 * (Q * x).dot(x) + c.dot(x);
        }

        return minimum;
    }
} // namespace sdqp

#endif
