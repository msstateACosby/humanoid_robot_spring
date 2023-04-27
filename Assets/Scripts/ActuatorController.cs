using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Mujoco;


public class ActuatorController : ActuatorComponent
{
    // Reference value for the overall ActionSpec of the combined IActuators associated with the component
    public override ActionSpec ActionSpec => new ActionSpec(numContinuousActions:1);

    // We need to add the actuator from the Editor
    
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
    
    private ControlledMjActuator controlledMjActuator; // Wrapper object that applies actions from the Agent to the MjActuator.

    public override IActuator[] CreateActuators()
    {
        
        return new[] { 
            
            new ControlledMjActuator(left_hip_xz_actuator),
            new ControlledMjActuator(right_hip_xz_actuator),
            new ControlledMjActuator(left_knee_actuator),
            new ControlledMjActuator(right_knee_actuator),
            new ControlledMjActuator(left_ankle_actuator),
            new ControlledMjActuator(right_ankle_actuator)
            
        }; // Could create and return multiple IActuators if needed
    }

    private class ControlledMjActuator : IActuator
    {
        private ActionSpec actionSpec;
        public ActionSpec ActionSpec { get => actionSpec; }

        private MjActuator wrappedActuator;

        public string Name => wrappedActuator.name;
        


        // Used when no model is connected to the BrainParameters component.
        public void Heuristic(in ActionBuffers actionBuffersOut) { } 


        // The agent distributes its actions among all of its IActuators, each receiving a segment based on their ActionSpecs.
        public void OnActionReceived(ActionBuffers actionBuffers)
        {
            float control = actionBuffers.ContinuousActions[0];
            wrappedActuator.Control = control;
            
            //Debug.Log(Name + actionBuffers.ContinuousActions[0].ToString());
        }


        // Called at the end of an episode.
        public void ResetData() { } 


        // Limit the number of possible actions if taking discrete actions (e.g. prevent gridworld agent from walking into a wall), not applicable
        public void WriteDiscreteActionMask(IDiscreteActionMask actionMask){ } 


        public ControlledMjActuator(MjActuator actuator)
        {
            actionSpec = new ActionSpec(numContinuousActions: 1);
            wrappedActuator = actuator;
        }

    }

}
