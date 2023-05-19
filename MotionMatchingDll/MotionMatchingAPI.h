#pragma once

#ifdef MM_EXPORTS
#define MM_API __declspec(dllexport)
#else
#define MM_API __declspec(dllimport)
#endif  


extern "C"
{
#define MAX_TRAJECTORY_LENGTH 10
#define MAX_BONE_LENGTH 23
    
    typedef struct _ControllerInput
    {
        bool lmm_enabled = false;
        float simulation_velocity_halflife = 0.27f;
        float simulation_rotation_halflife = 0.27f;    
        float simulation_run_fwrd_speed = 4.0f;
        float simulation_run_side_speed = 3.0f;
        float simulation_run_back_speed = 2.5f;    
        float simulation_walk_fwrd_speed = 1.75f;
        float simulation_walk_side_speed = 1.5f;
        float simulation_walk_back_speed = 1.25f;
        float inertialize_blending_halflife = 0.0f;            
        float feature_weight_foot_position = 0.75f;
        float feature_weight_foot_velocity = 1.0f;
        float feature_weight_hip_velocity = 1.0f;
        float feature_weight_trajectory_positions = 1.0f;
        float feature_weight_trajectory_directions = 1.5f;        
        bool synchronization_enabled = false;
        float synchronization_data_factor = 1.0f;        
        bool adjustment_enabled = true;
        bool adjustment_by_velocity_enabled = true;
        float adjustment_position_halflife = 0.1f;
        float adjustment_rotation_halflife = 0.2f;        
        bool clamping_enabled = true;
        float clamping_max_distance = 0.15f;
        float clamping_max_angle = 0.5f * 3.1415926;        
        bool ik_enabled = true;
        float ik_unlock_radius = 0.2f;
        float ik_blending_halflife = 0.1f;        
    }ControllerInput;

    typedef struct _ControllerOutput
    {
        float simulation_position[3];
        float simulation_rotation[4];

        int trajectory_length;
        float trajectory_positions[3 * MAX_TRAJECTORY_LENGTH];
        float trajectory_rotations[4 * MAX_TRAJECTORY_LENGTH];
        
        int bone_length;
        float bone_positions[3 * MAX_BONE_LENGTH];
        float bone_rotations[4 * MAX_BONE_LENGTH];
        float global_bone_positions[3 * MAX_BONE_LENGTH];
        float global_bone_rotations[4 * MAX_BONE_LENGTH];
    }ControllerOutput;

    typedef struct _ControllerPose
    {
        int bone_length;
        int bone_parents[MAX_BONE_LENGTH];
        float bone_rest_positions[3 * MAX_BONE_LENGTH];
        float bone_rest_rotations[4 * MAX_BONE_LENGTH];
    }ControllerPose;
    
    // 0 创建控制器
    MM_API void* MM_CreateController();

    // 1 初始化控制器
    MM_API bool MM_InitController(void* controller, char* path);
    // 1 获得初始化姿态
    MM_API void MM_ControllerGetPose(void* controller, ControllerPose* pose);

    // 2 获取控制器输入
    MM_API void MM_ControllerGetInput(void* controller, ControllerInput* input);
    // 2 设置控制器输入
    MM_API void MM_ControllerSetInput(void* controller, ControllerInput* input);
    // 2 移动控制器
    MM_API void MM_MoveController(void* controller, float x, float y, float z);

    // 3 更新控制器
    MM_API void MM_UpdateController(void* controller, float dt);

    // 4 获取控制器输出
    MM_API void MM_ControllerGetOutput(void* controller, ControllerOutput* output);

    // 5 删除控制器
    MM_API void MM_DeleteController(void* controller);
}

