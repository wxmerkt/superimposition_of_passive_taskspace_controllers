#ifndef STACK_OF_PASSIVE_CONTROLLERS_FRACTAL_IMPEDANCE_CONTROLLER_HPP_
#define STACK_OF_PASSIVE_CONTROLLERS_FRACTAL_IMPEDANCE_CONTROLLER_HPP_

#include <Eigen/Dense>

namespace stack_of_passive_controllers_controller
{
inline double sign(double x)
{
    if (x > 0)
        return +1.0;
    else if (x == 0)
        return 0.0;
    else
        return -1.0;
}

typedef Eigen::Matrix<double, 6, 1> Vector6d;

struct PassiveController
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PassiveController(const std::string& link_name_in,
                      const Eigen::Ref<const Vector6d>& Wmax_in,
                      const Eigen::Ref<const Vector6d>& Errb_in,
                      const Eigen::Ref<const Vector6d>& Err0_in,
                      const Eigen::Ref<const Vector6d>& K0_in)
        : link_name(link_name_in),
          controller_name(link_name_in),
          Wmax(Wmax_in),
          Errb(Errb_in),
          Err0(Err0_in),
          K0(K0_in),
          Wmax_scale(1.0),
          Wmax_init_(Wmax_in),
          K0_init_(K0_in)
    {
    }

    // Returns F, xmax
    std::pair<Vector6d, Vector6d> FractalImpedanceControl(const Eigen::Ref<const Vector6d>& Xmax, const Eigen::Ref<const Vector6d>& dX, const Eigen::Ref<const Vector6d>& Err)
    {
        F_.setZero();
        xmax_.setZero();

        // Update scaled variables
        Wmax = Wmax_scale * Wmax_init_;
        K0 = Wmax_scale * K0_init_;

        for (int i = 0; i < 6; ++i)
        {
            constexpr double capacitor_charge = 20.;
            const double b = (Errb(i) - Err0(i)) / capacitor_charge;

            constexpr double cartesian_velocity_tolerance = 1e-2;
            if (sign(Err(i)) == -sign(dX(i)) || std::abs(dX(i)) < cartesian_velocity_tolerance)
            {
                xmax_(i) = Err(i);
                if (std::abs(Err(i)) < Err0(i))
                {
                    F_(i) = K0(i) * Err(i);
                }
                else
                {
                    F_(i) = sign(Err(i)) * ((1. - std::exp(-(std::abs(Err(i)) - Err0(i)) / b)) * (Wmax(i) - K0(i) * Err0(i)) + K0(i) * Err0(i));
                }
            }
            else
            {
                if (std::isfinite(Xmax(i)))
                {
                    xmax_(i) = Xmax(i);
                }
                else
                {
                    xmax_(i) = Err(i);
                }

                double xmid = xmax_(i) / 2.;
                if (std::abs(xmid) < 1e-6)
                {
                    F_(i) = 0.;
                    continue;
                }
                // double AbsXmax = std::abs(xmax_(i));

                double U;
                if (std::abs(xmax_(i)) < Err0(i))
                {
                    U = 0.5 * K0(i) * xmax_(i) * xmax_(i);
                }
                else
                {
                    U = Wmax(i) * std::abs(xmax_(i)) - Wmax(i) * Err0(i) + (K0(i) * Err0(i) * Err0(i)) / 2. -
                        b * (Wmax(i) - K0(i) * Err0(i)) +
                        b * std::exp(-(std::abs(xmax_(i)) - Err0(i)) / b) * (Wmax(i) - K0(i) * Err0(i));
                }

                double Kout = U / (xmid * xmid);
                F_(i) = -Kout * (xmid - Err(i));

                if (std::abs(F_(i)) > Wmax(i))
                {
                    F_(i) = -sign(xmid - Err(i)) * Wmax(i);
                }
            }
        }

        return {F_, xmax_};
    }

    std::string link_name;
    std::string controller_name;
    Vector6d Wmax;
    Vector6d Errb;
    Vector6d Err0;
    Vector6d K0;
    Eigen::Vector3d link_offset = Eigen::Vector3d::Zero();
    KDL::Frame link_offset_frame = KDL::Frame();

    void SetLinkOffset(const Eigen::Ref<const Eigen::Vector3d>& offset)
    {
        link_offset = offset;
        link_offset_frame = KDL::Frame(KDL::Vector(offset(0), offset(1), offset(2)));
    }

    Vector6d tmp_Xmax = Vector6d::Zero();
    double Wmax_scale = 1.0;
    // Original variables
    Vector6d Wmax_init_;
    Vector6d K0_init_;

    realtime_tools::RealtimeBuffer<KDL::Frame> target_pose;
    // Vector6d target_pose;
    Vector6d current_pose;

    bool base_for_orientation_control = false;
    double orientation_scale = 1.0;

private:
    Vector6d F_;
    Vector6d xmax_;
};
}  // namespace stack_of_passive_controllers_controller

#endif  // STACK_OF_PASSIVE_CONTROLLERS_FRACTAL_IMPEDANCE_CONTROLLER_HPP_
