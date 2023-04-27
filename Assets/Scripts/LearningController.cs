using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Mujoco;


public class LearningController : Agent
{
    // We can assign references to these joints from the Editor
    

    [SerializeField]
    public MjHingeJoint left_hip_xz_driven;
    [SerializeField]
    public MjHingeJoint right_hip_xz_driven;

    [SerializeField]
    public MjHingeJoint left_hip_xz_spring;
    [SerializeField]
    public MjHingeJoint right_hip_xz_spring;
    
    [SerializeField]
    public MjHingeJoint left_knee_driven;
    [SerializeField]
    public MjHingeJoint right_knee_driven;

    [SerializeField]
    public MjHingeJoint left_knee_spring;
    [SerializeField]
    public MjHingeJoint right_knee_spring;

    [SerializeField]
    public MjHingeJoint left_ankle_driven;
    [SerializeField]
    public MjHingeJoint right_ankle_driven;
    [SerializeField]
    public MjHingeJoint left_ankle_spring;
    [SerializeField]
    public MjHingeJoint right_ankle_spring;

    [SerializeField]
    public MjHingeJoint left_stabalizer;
    [SerializeField]
    public MjHingeJoint right_stabalizer;

    [SerializeField]
    public Transform robot;
    public MjFreeJoint robot_joint;
    double[] positions = new double[7];
    
    // Start is called before the first frame update
    void Start()
    {
        
        
        
        
    }
    public unsafe void grabFreeJoint ()
    {
        if (!(MjScene.InstanceExists && MjScene.Instance.Data!=null)) Debug.LogWarning("OH no");
        var data = MjScene.Instance.Data;
        positions[0] = data->qpos[robot_joint.QposAddress]; 
        positions[1] = data->qpos[robot_joint.QposAddress + 1];
        positions[2] = data->qpos[robot_joint.QposAddress + 2];
        positions[3] = data->qpos[robot_joint.QposAddress + 3];
        positions[4] = data->qpos[robot_joint.QposAddress + 4];
        positions[5] = data->qpos[robot_joint.QposAddress + 5];
        positions[6] = data->qpos[robot_joint.QposAddress + 6];
    }
    public override void Initialize()
    {
        base.Initialize();
        //grabFreeJoint();
    }

    // Since we are accessing memory shared with the MuJoCo simulation we have to do it in an "unsafe" context (You may need to enable this).
    public unsafe override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        // In case this is the first frame and the MuJoCo simulation didn't start yet, 
        // we know we will start in the correct state so we can skip it.
        if (!(MjScene.InstanceExists && MjScene.Instance.Data!=null)) return;
        
        // Get the reference to the bindings of the mjData structure https://mujoco.readthedocs.io/en/latest/APIreference.html#mjdata
        var data = MjScene.Instance.Data;

        data->qpos[robot_joint.QposAddress] = 0;
        data->qpos[robot_joint.QposAddress + 1] = 0;
        data->qpos[robot_joint.QposAddress + 2] = 1.5;
        data->qpos[robot_joint.QposAddress + 3] = 0;
        data->qpos[robot_joint.QposAddress + 4] = 0;
        data->qpos[robot_joint.QposAddress + 5] = 0;
        data->qpos[robot_joint.QposAddress + 6] = 0;
        data->qvel[robot_joint.DofAddress] = 0;
        data->qvel[robot_joint.DofAddress + 1] = 0;
        data->qvel[robot_joint.DofAddress + 2] = 0;
        data->qvel[robot_joint.DofAddress + 3] = 0;
        data->qvel[robot_joint.DofAddress + 4] = 0;
        data->qvel[robot_joint.DofAddress + 5] = 0;
        // Reset kinematics to 0
        //pos
        
        data->qpos[left_hip_xz_driven.QposAddress] = 0;
        data->qpos[right_hip_xz_driven.QposAddress] = 0;
        data->qpos[left_knee_driven.QposAddress] = 0;
        data->qpos[right_knee_driven.QposAddress] = 0;
        data->qpos[left_ankle_driven.QposAddress] = 0;
        data->qpos[right_ankle_driven.QposAddress] = 0;
        /*
        
        data->qpos[left_hip_xz_spring.QposAddress] = 0;
        data->qpos[right_hip_xz_spring.QposAddress] = 0;
        data->qpos[left_knee_spring.QposAddress] = 0;
        data->qpos[right_knee_spring.QposAddress] = 0;
        data->qpos[left_ankle_spring.QposAddress] = 0;
        data->qpos[right_ankle_spring.QposAddress] = 0;
        */
        data->qpos[left_stabalizer.QposAddress] = 0;
        data->qpos[right_stabalizer.QposAddress] = 0;
        
        
        //vel
        
        data->qvel[left_hip_xz_driven.DofAddress] = 0;
        data->qvel[right_hip_xz_driven.DofAddress] = 0;
        data->qvel[left_knee_driven.DofAddress] = 0;
        data->qvel[right_knee_driven.DofAddress] = 0;
        data->qvel[left_ankle_driven.DofAddress] = 0;
        data->qvel[right_ankle_driven.DofAddress] = 0;
        /*
        
        data->qvel[left_hip_xz_spring.DofAddress] = 0;
        data->qvel[right_hip_xz_spring.DofAddress] = 0;
        data->qvel[left_knee_spring.DofAddress] = 0;
        data->qvel[right_knee_spring.DofAddress] = 0;
        data->qvel[left_ankle_spring.DofAddress] = 0;
        data->qvel[right_ankle_spring.DofAddress] = 0;
        */
        data->qvel[left_stabalizer.DofAddress] = 0;
        data->qvel[right_stabalizer.DofAddress] = 0;
        

        //acc
        
        data->qacc[left_hip_xz_driven.DofAddress] = 0;
        data->qacc[right_hip_xz_driven.DofAddress] = 0;
        data->qacc[left_knee_driven.DofAddress] = 0;
        data->qacc[right_knee_driven.DofAddress] = 0;
        data->qacc[left_ankle_driven.DofAddress] = 0;
        data->qacc[right_ankle_driven.DofAddress] = 0;
        /*
        
        data->qacc[left_hip_xz_spring.DofAddress] = 0;
        data->qacc[right_hip_xz_spring.DofAddress] = 0;
        data->qacc[left_knee_spring.DofAddress] = 0;
        data->qacc[right_knee_spring.DofAddress] = 0;
        data->qacc[left_ankle_spring.DofAddress] = 0;
        data->qacc[right_ankle_spring.DofAddress] = 0;
        */
        data->qacc[left_stabalizer.DofAddress] = 0;
        data->qacc[right_stabalizer.DofAddress] = 0;
        

        
        //Debug.Log("REEEEEEEEEEEEEEE");
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

