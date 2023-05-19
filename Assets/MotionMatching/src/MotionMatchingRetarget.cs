using System.Collections.Generic;
using UnityEngine;
using UnityEngine.PlayerLoop;

namespace FollowMe.Runtime.MotionMatching
{
    public struct AvatarBoneTransform
    {
        public ControllerBones boneIndex;
        public Transform transform;
        
        public Vector3 initPosition;
        public Quaternion initRotation;

        public AvatarBoneTransform(Transform t, ControllerBones b)
        {
            boneIndex = b;
            transform = t;
            
            if (transform)
            {
                initPosition = transform.position;
                initRotation = transform.rotation;
            }
            else
            {
                initPosition = Vector3.one;
                initRotation = Quaternion.identity;
            }
        }
        
        public bool IsValid => transform;
    }
    
    
    public class AvatarAnimator
    {
        public AvatarBoneTransform[] boneTransforms = new AvatarBoneTransform[(int)ControllerBones.Bone_Count];

        public void Init(Animator animator, ControllerPose controllerPose, Dictionary<ControllerBones, HumanBodyBones> bonesMap)
        {
            for (int i = 0; i < (int)ControllerBones.Bone_Count; i++)
            {
                ControllerBones bone = (ControllerBones) i;
                
                Transform transform = null;
                if (bone == ControllerBones.Bone_Entity)
                {
                    transform = animator.transform;
                }else if (bonesMap.TryGetValue(bone, out HumanBodyBones humanBodyBone))
                {
                    transform = animator.GetBoneTransform(humanBodyBone);
                }
                
                boneTransforms[i] = new AvatarBoneTransform(transform, bone);
            }
            
            AlignSkeleton(controllerPose);
            
            
            for (int i = 0; i < (int)ControllerBones.Bone_Count; i++)
            {
                if (boneTransforms[i].transform)
                {
                    boneTransforms[i].initPosition = boneTransforms[i].transform.position;
                    boneTransforms[i].initRotation = boneTransforms[i].transform.rotation;
                }
            }
        }
        
        
            
        public void AlignSkeleton(ControllerPose controllerPose)
        {
            // AlignBone(controllerPose, ControllerBones.Bone_Spine, ControllerBones.Bone_Spine1);
            // AlignBone(controllerPose, ControllerBones.Bone_Spine1, ControllerBones.Bone_Spine2);
            // AlignBone(controllerPose, ControllerBones.Bone_Spine2, ControllerBones.Bone_Neck);
            // AlignBone(controllerPose, ControllerBones.Bone_Neck, ControllerBones.Bone_Head);
            
            AlignBone(controllerPose, ControllerBones.Bone_LeftShoulder, ControllerBones.Bone_LeftArm);
            AlignBone(controllerPose, ControllerBones.Bone_LeftArm, ControllerBones.Bone_LeftForeArm);
            AlignBone(controllerPose, ControllerBones.Bone_LeftForeArm, ControllerBones.Bone_LeftHand);
            
            AlignBone(controllerPose, ControllerBones.Bone_RightShoulder, ControllerBones.Bone_RightArm);
            AlignBone(controllerPose, ControllerBones.Bone_RightArm, ControllerBones.Bone_RightForeArm);
            AlignBone(controllerPose, ControllerBones.Bone_RightForeArm, ControllerBones.Bone_RightHand);
            
            AlignBone(controllerPose, ControllerBones.Bone_LeftUpLeg, ControllerBones.Bone_LeftLeg);
            AlignBone(controllerPose, ControllerBones.Bone_LeftLeg, ControllerBones.Bone_LeftFoot);
            // AlignBone(controllerPose, ControllerBones.Bone_LeftFoot, ControllerBones.Bone_LeftToe);
            
            AlignBone(controllerPose, ControllerBones.Bone_RightUpLeg, ControllerBones.Bone_RightLeg);
            AlignBone(controllerPose, ControllerBones.Bone_RightLeg, ControllerBones.Bone_RightFoot);
            // AlignBone(controllerPose, ControllerBones.Bone_RightFoot, ControllerBones.Bone_RightToe);
        }

        private void AlignBone(ControllerPose controllerPose, ControllerBones bone1, ControllerBones[] bone2List)
        {
            Transform targetBone1 = boneTransforms[(int)bone1].transform;
            
            Vector3 sourceBone2Avg = Vector3.zero;
            Vector3 targetBone2Avg = Vector3.zero;
            foreach (var bone2 in bone2List)
            {
                sourceBone2Avg += controllerPose.bone_rest_positions[(int)bone2];
                targetBone2Avg += boneTransforms[(int)bone2].transform.position;
            }
            sourceBone2Avg /= bone2List.Length;
            targetBone2Avg /= bone2List.Length;
            
            if (targetBone1)
            {
                Vector3 sourceDir = sourceBone2Avg - controllerPose.bone_rest_positions[(int)bone1];
                Vector3 targetDir = targetBone2Avg - targetBone1.position;
                Vector3 localTargetDir = targetBone1.transform.InverseTransformDirection(targetDir);
                Vector3 localSourceDir = targetBone1.transform.InverseTransformDirection(sourceDir);
                // Debug.DrawLine(sourceBone1.position, sourceBone2.position, Color.yellow, 10.0f);
                // Debug.DrawLine(targetBone2.position, targetBone1.position, Color.yellow, 10.0f);
                Quaternion rotation = Quaternion.FromToRotation(localTargetDir, localSourceDir);
                targetBone1.localRotation = targetBone1.localRotation * rotation;
            }
        }
        
        private void AlignBone(ControllerPose controllerPose, ControllerBones bone1, ControllerBones bone2)
        {
            Transform targetBone1 = boneTransforms[(int)bone1].transform;
            Transform targetBone2 = boneTransforms[(int)bone2].transform;
            
            if (targetBone1 && targetBone2)
            {
                Vector3 sourceDir = controllerPose.bone_rest_positions[(int)bone2] - controllerPose.bone_rest_positions[(int)bone1];
                Vector3 targetDir = targetBone2.position - targetBone1.position;
                Vector3 localTargetDir = targetBone1.transform.InverseTransformDirection(targetDir);
                Vector3 localSourceDir = targetBone1.transform.InverseTransformDirection(sourceDir);
                // Debug.DrawLine(sourceBone1.position, sourceBone2.position, Color.yellow, 10.0f);
                // Debug.DrawLine(targetBone2.position, targetBone1.position, Color.yellow, 10.0f);
                Quaternion rotation = Quaternion.FromToRotation(localTargetDir, localSourceDir);
                targetBone1.localRotation = targetBone1.localRotation * rotation;
            }
        }
    }

    public class MotionMatchingRetarget
    {
        private bool isInit = false;
        public AvatarAnimator targetAvatarAnimator;
        public ControllerPose sourceControllerPose;

        private static Dictionary<ControllerBones, HumanBodyBones> bonesMap = new Dictionary<ControllerBones, HumanBodyBones>
        {
            {ControllerBones.Bone_Hips, HumanBodyBones.Hips},
            {ControllerBones.Bone_Spine, HumanBodyBones.Spine},
            {ControllerBones.Bone_Spine1, HumanBodyBones.Chest},
            {ControllerBones.Bone_Spine2, HumanBodyBones.UpperChest},
            {ControllerBones.Bone_Neck, HumanBodyBones.Neck},
            {ControllerBones.Bone_Head, HumanBodyBones.Head},
            {ControllerBones.Bone_LeftShoulder, HumanBodyBones.LeftShoulder},
            {ControllerBones.Bone_LeftArm, HumanBodyBones.LeftUpperArm},
            {ControllerBones.Bone_LeftForeArm, HumanBodyBones.LeftLowerArm},
            {ControllerBones.Bone_LeftHand, HumanBodyBones.LeftHand},
            {ControllerBones.Bone_RightShoulder, HumanBodyBones.RightShoulder},
            {ControllerBones.Bone_RightArm, HumanBodyBones.RightUpperArm},
            {ControllerBones.Bone_RightForeArm, HumanBodyBones.RightLowerArm},
            {ControllerBones.Bone_RightHand, HumanBodyBones.RightHand},
            {ControllerBones.Bone_LeftUpLeg, HumanBodyBones.LeftUpperLeg},
            {ControllerBones.Bone_LeftLeg, HumanBodyBones.LeftLowerLeg},
            {ControllerBones.Bone_LeftFoot, HumanBodyBones.LeftFoot},
            {ControllerBones.Bone_LeftToe, HumanBodyBones.LeftToes},
            {ControllerBones.Bone_RightUpLeg, HumanBodyBones.RightUpperLeg},
            {ControllerBones.Bone_RightLeg, HumanBodyBones.RightLowerLeg},
            {ControllerBones.Bone_RightFoot, HumanBodyBones.RightFoot},
            {ControllerBones.Bone_RightToe, HumanBodyBones.RightToes},
        };


        public void Init(GameObject targetAvatarRoot, ControllerPose controllerPose)
        {
            isInit = false;
            if (!targetAvatarRoot)
            {
                return;
            }

            Animator animator = targetAvatarRoot.GetComponent<Animator>();
            if (!animator)
            {
                return;
            }

            sourceControllerPose = controllerPose;
            targetAvatarAnimator = new AvatarAnimator();
            targetAvatarAnimator.Init(animator, controllerPose, bonesMap);


            isInit = true;
        }

        public void Update(ControllerOutput controllerOutput)
        {
            if (!isInit || targetAvatarAnimator == null)
            {
                return;
            }
            
            for (int i = (int)ControllerBones.Bone_Hips; i < (int)ControllerBones.Bone_Count; i++)
            {
                if (targetAvatarAnimator.boneTransforms[i].transform == null)
                {
                    continue;
                }
                
                Quaternion deltaRotation = controllerOutput.global_bone_rotations[i] * Quaternion.Inverse(sourceControllerPose.bone_rest_rotations[i]); 
                Quaternion rotation = deltaRotation * targetAvatarAnimator.boneTransforms[i].initRotation;
                targetAvatarAnimator.boneTransforms[i].transform.rotation = rotation;
                
                // targetAvatarAnimator.boneTransforms[i].transform.rotation = controllerOutput.global_bone_rotations[i];
            }
            
            int entityBoneIndex = (int)ControllerBones.Bone_Entity;
            AvatarBoneTransform entityTransform = targetAvatarAnimator.boneTransforms[entityBoneIndex];
            entityTransform.transform.position = controllerOutput.global_bone_positions[entityBoneIndex] - sourceControllerPose.bone_rest_positions[entityBoneIndex] + entityTransform.initPosition;
            
            
            // align toe height
            int leftFootBoneIndex = (int)ControllerBones.Bone_LeftFoot;
            int rightFootBoneIndex = (int)ControllerBones.Bone_RightFoot;
            AvatarBoneTransform leftFootTransform = targetAvatarAnimator.boneTransforms[leftFootBoneIndex];
            AvatarBoneTransform rightFootTransform = targetAvatarAnimator.boneTransforms[rightFootBoneIndex];
            float leftFootPos = controllerOutput.global_bone_positions[leftFootBoneIndex].y;
            float rightFootPos = controllerOutput.global_bone_positions[rightFootBoneIndex].y;

            float height = leftFootPos > rightFootPos ? rightFootPos - rightFootTransform.transform.position.y : leftFootPos - leftFootTransform.transform.position.y;
            entityTransform.transform.position = new Vector3(entityTransform.transform.position.x, entityTransform.initPosition.y + height, entityTransform.transform.position.z);
            
        }
        
    }
}