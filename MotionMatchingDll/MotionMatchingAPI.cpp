#include "MotionMatchingAPI.h"
#include "src/controller.h"

void* MM_CreateController()
{
    return new controller();
}

void MM_DeleteController(void* c)
{
    delete (class controller*)c;
}

bool MM_InitController(void* c, char* path)
{
    return ((class controller*)c)->init(path);
}

void MM_UpdateController(void* c, float dt)
{
    ((class controller*)c)->update(dt);
}

void MM_MoveController(void* c, float x, float y, float z)
{
    ((class controller*)c)->move(vec3(x, y, z));
}

void ToVec(vec3 inV, float* outV)
{
    outV[0] = inV.x;
    outV[1] = inV.y;
    outV[2] = inV.z;
}

void ToQuat(quat inQ, float* outQ)
{
    outQ[0] = inQ.x;
    outQ[1] = inQ.y;
    outQ[2] = inQ.z;
    outQ[3] = inQ.w;
}

int MaxInt(int a, int b)
{
    return a > b ? a : b;
}

int MinInt(int a, int b)
{
    return a < b ? a : b;
}

void MM_ControllerSetInput(void* c, ControllerInput* input)
{
    controller* _controller = (class controller*)c;
    _controller->lmm_enabled = input->lmm_enabled;
    _controller->simulation_velocity_halflife = input->simulation_velocity_halflife;
    _controller->simulation_rotation_halflife = input->simulation_rotation_halflife;
    _controller->simulation_run_fwrd_speed = input->simulation_run_fwrd_speed;
    _controller->simulation_run_side_speed = input->simulation_run_side_speed;
    _controller->simulation_run_back_speed = input->simulation_run_back_speed;
    _controller->simulation_walk_fwrd_speed = input->simulation_walk_fwrd_speed;
    _controller->simulation_walk_side_speed = input->simulation_walk_side_speed;
    _controller->simulation_walk_back_speed = input->simulation_walk_back_speed;
    _controller->inertialize_blending_halflife = input->inertialize_blending_halflife;
    _controller->feature_weight_foot_position = input->feature_weight_foot_position;
    _controller->feature_weight_foot_velocity = input->feature_weight_foot_velocity;
    _controller->feature_weight_hip_velocity = input->feature_weight_hip_velocity;
    _controller->feature_weight_trajectory_positions = input->feature_weight_trajectory_positions;
    _controller->feature_weight_trajectory_directions = input->feature_weight_trajectory_directions;
    _controller->synchronization_enabled = input->synchronization_enabled;
    _controller->synchronization_data_factor = input->synchronization_data_factor;
    _controller->adjustment_enabled = input->adjustment_enabled;
    _controller->adjustment_by_velocity_enabled = input->adjustment_by_velocity_enabled;
    _controller->adjustment_position_halflife = input->adjustment_position_halflife;
    _controller->adjustment_rotation_halflife = input->adjustment_rotation_halflife;
    _controller->clamping_enabled = input->clamping_enabled;
    _controller->clamping_max_distance = input->clamping_max_distance;
    _controller->clamping_max_angle = input->clamping_max_angle;
    _controller->ik_enabled = input->ik_enabled;
    _controller->ik_unlock_radius = input->ik_unlock_radius;
    _controller->ik_blending_halflife = input->ik_blending_halflife;  
}

void MM_ControllerGetInput(void* c, ControllerInput* input)
{
    controller* _controller = (class controller*)c;
    input->lmm_enabled = _controller->lmm_enabled;
    input->simulation_velocity_halflife = _controller->simulation_velocity_halflife;
    input->simulation_rotation_halflife = _controller->simulation_rotation_halflife;
    input->simulation_run_fwrd_speed = _controller->simulation_run_fwrd_speed;
    input->simulation_run_side_speed = _controller->simulation_run_side_speed;
    input->simulation_run_back_speed = _controller->simulation_run_back_speed;
    input->simulation_walk_fwrd_speed = _controller->simulation_walk_fwrd_speed;
    input->simulation_walk_side_speed = _controller->simulation_walk_side_speed;
    input->simulation_walk_back_speed = _controller->simulation_walk_back_speed;
    input->inertialize_blending_halflife = _controller->inertialize_blending_halflife;
    input->feature_weight_foot_position = _controller->feature_weight_foot_position;
    input->feature_weight_foot_velocity = _controller->feature_weight_foot_velocity;
    input->feature_weight_hip_velocity = _controller->feature_weight_hip_velocity;
    input->feature_weight_trajectory_positions = _controller->feature_weight_trajectory_positions;
    input->feature_weight_trajectory_directions = _controller->feature_weight_trajectory_directions;
    input->synchronization_enabled = _controller->synchronization_enabled;
    input->synchronization_data_factor = _controller->synchronization_data_factor;
    input->adjustment_enabled = _controller->adjustment_enabled;
    input->adjustment_by_velocity_enabled = _controller->adjustment_by_velocity_enabled;
    input->adjustment_position_halflife = _controller->adjustment_position_halflife;
    input->adjustment_rotation_halflife = _controller->adjustment_rotation_halflife;
    input->clamping_enabled = _controller->clamping_enabled;
    input->clamping_max_distance = _controller->clamping_max_distance;
    input->clamping_max_angle = _controller->clamping_max_angle;
    input->ik_enabled = _controller->ik_enabled;
    input->ik_unlock_radius = _controller->ik_unlock_radius;
    input->ik_blending_halflife = _controller->ik_blending_halflife;
}

void MM_ControllerGetOutput(void* c, ControllerOutput* output)
{
    controller* _controller = (class controller*)c;

    ToVec(_controller->simulation_position, output->simulation_position);
    ToQuat(_controller->simulation_rotation, output->simulation_rotation);

    output->trajectory_length = MinInt(_controller->trajectory_positions.size, MAX_TRAJECTORY_LENGTH);    
    for (int i = 0; i< output->trajectory_length; i++)
    {
        ToVec(_controller->trajectory_positions(i), output->trajectory_positions + i * 3);
        ToQuat(_controller->trajectory_rotations(i), output->trajectory_rotations + i * 4);
    }

    output->bone_length = MinInt(_controller->bone_positions.size, MAX_BONE_LENGTH);
    for (int i = 0; i< output->bone_length; i++)
    {
        ToVec(_controller->bone_positions(i), output->bone_positions + i * 3);
        ToQuat(_controller->bone_rotations(i), output->bone_rotations + i * 4);
        ToVec(_controller->global_bone_positions(i), output->global_bone_positions + i * 3);
        ToQuat(_controller->global_bone_rotations(i), output->global_bone_rotations + i * 4);        
    }
}

void MM_ControllerGetPose(void* c, ControllerPose* pose)
{
    controller* _controller = (class controller*)c;

    pose->bone_length = MinInt(_controller->character_data.bone_rest_positions.size, MAX_BONE_LENGTH);    
    for (int i = 0; i< pose->bone_length; i++)
    {
        pose->bone_parents[i] = _controller->db.bone_parents(i);
        ToVec(_controller->character_data.bone_rest_positions(i), pose->bone_rest_positions + i * 3);
        ToQuat(_controller->character_data.bone_rest_rotations(i), pose->bone_rest_rotations + i * 4);
    }    
}
