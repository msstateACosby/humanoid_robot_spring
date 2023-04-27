using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using System;
using System.Linq;
using Mujoco;


// A base class we inherit from. You can use this as well in your own rewards.
public abstract class RewardProvider: MonoBehaviour
{
    [SerializeField]
    protected Agent agent;
    [SerializeField]
    protected Transform robot_position;

    // Note that there is no guarantee by default on the execution order of the FixedUpdate() of different components. 
    // This also affects the Agent; we don't know if the MuJoCo physics step happens first or the agent one, or the reward.
    // Additional information on guaraneeing synchronisation will be discussed in a later tutorial.
    private void FixedUpdate()
    {
        
        agent.AddReward(Reward);
        if (robot_position.transform.localPosition.y < 0.5f)
        {
            //Debug.Log("Should be reseting");
            agent.EndEpisode();
        }
    }

    public abstract float Reward { get; }
}

/// <summary>
/// 
/// </summary>
public class BipedRewards : RewardProvider
{
    
    [SerializeField]
    private MjActuator left_hip_xz_actuator;
    [SerializeField]
    private MjActuator right_hip_xz_actuator;
    [SerializeField]
    private MjActuator left_knee_actuator;
    [SerializeField]
    private MjActuator right_knee_actuator;
    [SerializeField]
    public MjActuator left_ankle_actuator;
    [SerializeField]
    public MjActuator right_ankle_actuator;
    

    

   

    public override float Reward { get => GetStateReward(); }
    float past_z = 0;

    // Our desired state is forward relative to starting position
    //with robot not falling down
    float GetStateReward()
    {
        float robot_z = robot_position.position.z; //Mathf.Max(0, robot_position.position.z/5);
        float robot_x = robot_position.position.x;
        
        float forward_reward = robot_z - past_z;
        past_z = robot_z;
        float height = robot_position.position.y;
        float still_standing = Tolerance(height, bounds:(1.2f,1.6f), margin: .5f);
        float close_to_center = Tolerance(robot_z, bounds: (0f,0f), margin: 1f) * Tolerance(robot_x, bounds: (0f,0f), margin: 1f);
        float regularization = (
                                regularize(left_hip_xz_actuator)  * regularize(right_hip_xz_actuator)
                                 * regularize(left_knee_actuator)  * regularize(right_knee_actuator)
                                 * regularize(left_ankle_actuator)  * regularize(right_ankle_actuator) );
        
        //Debug.Log(forward * still_standing * regularization);
        return close_to_center * still_standing * regularization; // Multiply, so we need to satisfy all three at the same time
        
    }

    // We want to use as little force as possible to get there.
    
    
    float regularize(MjActuator actuator)
    {
        var control = actuator.Control;
        var smallControl = Tolerance(control, bounds:(0f, 0f), margin: 1, valueAtMargin: 0, sigmoid: "quadratic");
        smallControl = (4 + smallControl) / 5;
        return smallControl;
    }
    
    // Non-linearity to process the actions that penalize being outside the given bounds
    private static float Tolerance(float x, (float, float) bounds, float margin=0, 
                                    string sigmoid="gaussian", float valueAtMargin=0.1f)
    {
        (float lower, float upper) = bounds;

        if(lower > upper)
        {
            throw new ArgumentException("Lower bound must be <= upper bound.");
        }

        if(margin < 0)
        {
            throw new ArgumentException("'margin' must be non-negative");
        }

        bool inBounds = (lower <= x) && (x <= upper);

        float value;
        if (margin == 0)
        {
            value = inBounds? 1 : 0;
        }
        else
        {
            float d = (x < lower ? lower - x : x - upper) / margin;
            value = inBounds ? 1 : Sigmoids(d, valueAtMargin, sigmoid);
        }

        return value;
    }

    public static float Sigmoids(float x, float valueAt1, string sigmoid)
    {
        if (new[] { "cosine", "linear", "quadratic" }.Contains(sigmoid))
        {
            if (!(0f <= valueAt1 && valueAt1 < 1f))
            {
                throw new ArgumentException($"`value_at_1` must be nonnegative and smaller than 1, got{valueAt1}");
            }
            }
        else if(!(0<valueAt1 && valueAt1<1))
        {
            throw new ArgumentException($"`value_at_1` must be strictly between 0 and 1, got {valueAt1}");
        }

        switch(sigmoid)
        {
            case "gaussian":
                var scale = Mathf.Sqrt(-2 * Mathf.Log(valueAt1));
                return Mathf.Exp(-0.5f * x * x * scale * scale);

            case "quadratic":
                scale = Mathf.Sqrt(1 - valueAt1);
                var scaledX = x * scale;
                return Mathf.Abs(scaledX) < 1 ? 1 - (scaledX * scaledX) : 0;

            default:
                throw new NotImplementedException($"Unknown sigmoid type {sigmoid}");
        }

    }

    
}
