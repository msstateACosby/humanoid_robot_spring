using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Mujoco;
public class biped_agent : Agent
{
    [SerializeField] 
    MjHingeJoint left_hip_xy_actuated_joint;
    [SerializeField] 
    MjHingeJoint left_hip_xy_spring_joint;
    [SerializeField] 
    MjHingeJoint left_hip_xz_actuated_joint;
    [SerializeField] 
    MjHingeJoint left_hip_xz_spring_joint;
    [SerializeField] 
    MjHingeJoint left_knee_actuated_joint;
    [SerializeField] 
    MjHingeJoint left_knee_spring_joint;
    [SerializeField] 
    MjHingeJoint left_foot_xy_joint;
    [SerializeField]
    MjHingeJoint left_foot_xz_joint;

    [SerializeField] 
    MjHingeJoint right_hip_xy_actuated_joint;
    [SerializeField] 
    MjHingeJoint right_hip_xy_spring_joint;
    [SerializeField] 
    MjHingeJoint right_hip_xz_actuated_joint;
    [SerializeField] 
    MjHingeJoint right_hip_xz_spring_joint;
    [SerializeField] 
    MjHingeJoint right_knee_actuated_joint;
    [SerializeField] 
    MjHingeJoint right_knee_spring_joint;
    [SerializeField] 
    MjHingeJoint right_foot_xy_joint;
    [SerializeField]
    MjHingeJoint right_foot_xz_joint;


    
    // Start is called before the first frame update
    void Start()
    {
        
    }

     public unsafe override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        // In case this is the first frame and the MuJoCo simulation didn't start yet, 
        // we know we will start in the correct state so we can skip it.
        if (!(MjScene.InstanceExists && MjScene.Instance.Data!=null)) return;
        
        // Get the reference to the bindings of the mjData structure https://mujoco.readthedocs.io/en/latest/APIreference.html#mjdata
        var data = MjScene.Instance.Data;
        
        // Reset kinematics to 0
        data->qpos[left_hip_xy_actuated_joint.QposAddress] = 0;
        data->qpos[left_hip_xy_spring_joint.QposAddress] = 0;
        data->qpos[left_hip_xz_actuated_joint.QposAddress] = 0;
        data->qpos[left_hip_xz_spring_joint.QposAddress] = 0;
        data->qpos[left_knee_actuated_joint.QposAddress] = 0;
        data->qpos[left_knee_spring_joint.QposAddress] = 0;
        data->qpos[left_foot_xy_joint.QposAddress] = 0;
        data->qpos[left_foot_xz_joint.QposAddress] = 0;

        data->qpos[right_hip_xy_actuated_joint.QposAddress] = 0;
        data->qpos[right_hip_xy_spring_joint.QposAddress] = 0;
        data->qpos[right_hip_xz_actuated_joint.QposAddress] = 0;
        data->qpos[right_hip_xz_spring_joint.QposAddress] = 0;
        data->qpos[right_knee_actuated_joint.QposAddress] = 0;
        data->qpos[right_knee_spring_joint.QposAddress] = 0;
        data->qpos[right_foot_xy_joint.QposAddress] = 0;
        data->qpos[right_foot_xz_joint.QposAddress] = 0;
        
        
        // Reset velocites to 0
        data->qvel[left_hip_xy_actuated_joint.DofAddress] = 0;
        data->qvel[left_hip_xy_spring_joint.DofAddress] = 0;
        data->qvel[left_hip_xz_actuated_joint.DofAddress] = 0;
        data->qvel[left_hip_xz_spring_joint.DofAddress] = 0;
        data->qvel[left_knee_actuated_joint.DofAddress] = 0;
        data->qvel[left_knee_spring_joint.DofAddress] = 0;
        data->qvel[left_foot_xy_joint.DofAddress] = 0;
        data->qvel[left_foot_xz_joint.DofAddress] = 0;

        data->qvel[right_hip_xy_actuated_joint.DofAddress] = 0;
        data->qvel[right_hip_xy_spring_joint.DofAddress] = 0;
        data->qvel[right_hip_xz_actuated_joint.DofAddress] = 0;
        data->qvel[right_hip_xz_spring_joint.DofAddress] = 0;
        data->qvel[right_knee_actuated_joint.DofAddress] = 0;
        data->qvel[right_knee_spring_joint.DofAddress] = 0;
        data->qvel[right_foot_xy_joint.QposAddress] = 0;
        data->qvel[right_foot_xz_joint.DofAddress] = 0;

        // Reset velocites to 0
        data->qacc[left_hip_xy_actuated_joint.DofAddress] = 0;
        data->qacc[left_hip_xy_spring_joint.DofAddress] = 0;
        data->qacc[left_hip_xz_actuated_joint.DofAddress] = 0;
        data->qacc[left_hip_xz_spring_joint.DofAddress] = 0;
        data->qacc[left_knee_actuated_joint.DofAddress] = 0;
        data->qacc[left_knee_spring_joint.DofAddress] = 0;
        data->qacc[left_foot_xy_joint.DofAddress] = 0;
        data->qacc[left_foot_xz_joint.DofAddress] = 0;

        data->qacc[right_hip_xy_actuated_joint.DofAddress] = 0;
        data->qacc[right_hip_xy_spring_joint.DofAddress] = 0;
        data->qacc[right_hip_xz_actuated_joint.DofAddress] = 0;
        data->qacc[right_hip_xz_spring_joint.DofAddress] = 0;
        data->qacc[right_knee_actuated_joint.DofAddress] = 0;
        data->qacc[right_knee_spring_joint.DofAddress] = 0;
        data->qacc[right_foot_xy_joint.QposAddress] = 0;
        data->qacc[right_foot_xz_joint.DofAddress] = 0;
        
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);
        // If you wanted to collect observations from the Agent class, you can add them one by one to the sensor
        // Note that if you do this, and not via separate SensorComponents, you will have to update the BehaviourParameter's observation size
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // You could process your agent's actions directly here, and assign reward as well
    }
}
