#include "pinocchio/fwd.hpp"
#include <cmath>
#include <iostream>
#include "kinematics.h"

#include <fstream>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <ros/package.h>

#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/emanuele/ros_ws/src/locosim/robot_descriptions/ur_description"
#endif

using namespace std;
using namespace pinocchio;
using namespace ros;

const string urdf_filename = string("/home/emanuele/ros_ws/src/locosim/robot_urdf/ur5.urdf");
// const string urdf_filename = string("/home/emanuele/ros_ws/src/locosim/robot_urdf/generated_urdf/ur5.urdf");
const string srdf_filename = string("/home/emanuele/ros_ws/src/locosim/robot_urdf/ur5.srdf");
const string mesh_dir = string("/home/emanuele/ros_ws/src/locosim/robot_descriptions/ur_description/meshes/ur5e/");

const double eps = 1e-2;
const int IT_MAX = 1000;
const double DT = 1e-2;
const double damp = 1e-5;
const int JOINT_ID = 6;

jointValues ur5InversePinocchio(coordinates pe, rotMatrix re, jointValues current_joints)
{

    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    Data data(model);

    GeometryModel geom_model;
    geom_model = pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model, mesh_dir);
    // Add all possible collision pairs and remove the ones collected in the SRDF file
    geom_model.addAllCollisionPairs();
    // Load the reference configuration of the robots (this one should be collision free)
    pinocchio::srdf::loadReferenceConfigurations(model, srdf_filename); // the reference configuratio stored in the SRDF file is called half_sitting

    // Build the data associated to the geom_model
    GeometryData geom_data(geom_model); // contained the intermediate computations, like the placement of all the geometries with respect to the world frame

    const Model::ConfigVectorType &_q = current_joints;

    // And test all the collision pairs
    computeCollision(geom_model, geom_data, false);
    // cout << "Collision pairs:" << endl;
    for (size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
        const CollisionPair &cp = geom_model.collisionPairs[k];
        const hpp::fcl::CollisionResult &cr = geom_data.collisionResults[k];

        // std::cout << "collision pair: " << cp.first << " , " << cp.second << " - collision: ";
        // std::cout << (cr.isCollision() ? "yes" : "no") << std::endl;
    }
    const pinocchio::SE3 oMdes2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));
    const pinocchio::SE3 oMdes(re, pe);
    cout << "oMdes: " << oMdes << endl;
    cout << "oMdes2: " << oMdes2 << endl;

    Eigen::VectorXd q = current_joints;
    // cout
    //     << "neutral: " << q.transpose() << endl;

    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();

    // cout << "J: " << J << endl;

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model.nv);

    // print all the info of the joints
    for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left
                  << model.names[joint_id] << ": "
                  << std::fixed << std::setprecision(2)
                  << data.oMi[joint_id].translation().transpose()
                  << std::endl
                  << std::endl;
    for (int i = 0;; i++)
    {
        pinocchio::forwardKinematics(model, data, q);
        const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);

        // cout << "oMdes: " << oMdes << endl;
        // cout << "iMd: " << iMd << endl;
        err = pinocchio::log6(iMd).toVector(); // in joint frame

        if (err.norm() < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            break;
        }
        pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J); // J in joint frame
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
        J = -Jlog * J;
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model, q, v * DT);
        // if (!(i % 1000))
        //     std::cout << i << ": error = " << err.transpose() << std::endl;
    }

    if (success)
    {
        // std::cout << "Convergence achieved!" << std::endl;
        // for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        //     std::cout << std::setw(24) << std::left
        //               << model.names[joint_id] << ": "
        //               << std::fixed << std::setprecision(2)
        //               << data.oMi[joint_id].translation().transpose()
        //               << std::endl
        //               << std::endl;

        for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
        {
            cout << model.names[joint_id] << ": " << data.oMi[joint_id] << endl;
        }
    }
    else
    {
        // std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    }

    // std::cout << "\nresult: " << q.transpose() << std::endl;
    jointValues res = q;
    // std::cout << "\nfinal error: " << err.transpose() << std::endl
    //           << std::endl;

    return res;
}