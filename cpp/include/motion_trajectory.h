#include "kinematics.h"

using namespace Eigen;
using namespace std;

const bool debug_traj = true;
const bool error_code_debug = true;

const int steps = 20;

const double max_x = 1.5;
const double max_y = 0.275;
const double max_z = 0.885;

const double min_x = -1.5;
const double min_y = -1.5;
const double min_z = 0.0;

const double max_y_near_end_table = 0.125;
const double max_z_near_end_table = 0.73;

const double max_z_moving = 0.81;
const double max_z_moving_gripping = 0.78;

const double max_z_sidepick = 0.84;
const double max_z_moving_sidepick = 0.8;
const double max_z_moving_gripping_sidepick = 0.77;

const jointValues mainJointResetValues = (jointValues() << 0., -1.57, -2.5, 0., 0., 0.).finished();

const coordinates defaultCordArray[6] = {
    (coordinates() << 0.35, 0.03, 0.7).finished(),
    (coordinates() << 0.01, 0.03, 0.7).finished(),
    (coordinates() << -0.33, 0.03, 0.7).finished(),
    (coordinates() << -0.33, -0.29, 0.7).finished(),
    (coordinates() << 0.01, -0.29, 0.7).finished(),
    (coordinates() << 0.35, -0.29, 0.7).finished(),

};

bool check_singularity_collision(jointValues joints);
bool check_trajectory(vector<double *> traj, int step, bool pick_or_place, int *error_code, const coordinates &requested_cord, const rotMatrix &requested_rotation, const jointValues &init_joint, bool homing, bool side_pick, bool isGripping);

void ur5Trajectory(vector<double *> *Th, jointValues initial_position, jointValues final_position, int steps);
bool init_verify_trajectory(vector<double *> *Th, jointValues init_joint, jointValues final_joint, int steps, bool pick_or_place, const coordinates &requested_cord, const rotMatrix &requested_rotation, bool homing, bool side_pick, bool isGripping);

vector<double *> up_and_move(const coordinates &position, const rotMatrix &rotation, int steps, jointValues startJoint, bool side_pick, bool isGripping);
vector<double *> move_to_near_axis(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, jointValues startJoint, bool side_pick, bool isGripping);
vector<double *> reset_main_joint(const coordinates &position, const rotMatrix &rotation, int steps, jointValues startPosition, bool isGripping);
vector<double *> up_tray(jointValues startJoint, bool side_pick, bool isGripping);
vector<double *> calc_direct_traj_joint(const jointValues endJoint, bool pick_or_place, bool homing, jointValues startPosition, bool side_pick, bool isGripping);
vector<double *> calc_direct_traj_multiple_joint(vector<jointValues> endJoints, bool *pick_or_place, bool *homing, jointValues startPosition, bool *side_pick, bool isGripping);
vector<double *> calc_direct_traj(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, jointValues startPosition, bool side_pick, bool isGripping);
vector<double *> calc_direct_traj_multiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, jointValues startPosition, bool *side_pick, bool isGripping);
vector<double *> calc_traj(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag, jointValues startPosition, bool side_pick, bool isGripping, bool reset_flag);
vector<double *> calc_traj_multiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag, jointValues startJoint, bool *side_pick, bool isGripping);
vector<double *> move_through_homing(coordinates final_cord, rotMatrix rot, jointValues startJoint, bool side_pick, bool isGripping);

coordinates nearHoming(coordinates cord);
coordinates nearHomingRec(coordinates current_cord, coordinates defaultCord, double &nearhomingdist, coordinates &nearhomingcord);
rotMatrix get_rotation(double angle);