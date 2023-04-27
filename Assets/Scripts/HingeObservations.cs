using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Mujoco;
using System.Linq;

public class HingeObservations : SensorComponent
{
    // SensorComponents don't provide the readings themselves, instead they provide the Agent with a collection of ISensors
    // A SensorComponent can group multiple separate sensors, but in this case we only need one.
    public override ISensor[] CreateSensors()
    {
        
        return new ISensor[] { 
            
            new HingeSensor(left_hip_xz_driven),
            new HingeSensor(right_hip_xz_driven),
            new HingeSensor(left_knee_driven),
            new HingeSensor(right_knee_driven),
            new HingeSensor(left_ankle_driven),
            new HingeSensor(right_ankle_driven),

            /*
            new HingeSensor(left_ankle_spring),
            new HingeSensor(right_ankle_spring),
            new HingeSensor(left_hip_xz_spring),
            new HingeSensor(right_hip_xz_spring),
            new HingeSensor(left_knee_spring),
            new HingeSensor(right_knee_spring),
            */
            
            new HingeSensor(left_stabalizer),
            new HingeSensor(right_stabalizer),
            new VectorSensor(gyro),
            new VectorSensor(accel),
            
            };
    }

    // Instances of our custom ISensors
    private HingeSensor hingeSensor;
    public List<float> Observations => hingeSensor.Observations; // Exposed in case needed for visualisation

    

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
    public MjSiteVectorSensor gyro;
    [SerializeField]
    public MjSiteVectorSensor accel;
    static int i=0;

    private class HingeSensor : ISensor
    {

        MjHingeJoint hinge;
        int nameindex=0;

        float[] Velocities { get => new[] { hinge.Velocity }; } // Note that velocity is already in radians in the Plugin

        float[] ContinuousHingePositions // We separate the angle into 2 components so it is a continuous and doesn't jump from 359 deg to 0 deg.
        {
            get
            {
                float angle = hinge.Configuration * Mathf.Deg2Rad;
                var sin = Mathf.Sin(angle);
                var cos = Mathf.Cos(angle);
                //Debug.Log("getting");
                return new[] { sin, cos };
            }
        }

        float[] Positions { get => ContinuousHingePositions; } // We combine the hinge and slider position

        public List<float> Observations { get => Positions.Concat(Velocities).ToList(); } //Combine positions and velocities

        public int Write(ObservationWriter writer)
        {
            writer.AddList(Observations);
            return 3; // Return number of written observations (2 for hinge pos, 1 for hinge vel)
        }

        // Only needed for visual observations (e.g. camera feed)
        public byte[] GetCompressedObservation()
        {
            throw new NotImplementedException();
        }

        public CompressionSpec GetCompressionSpec()
        {
            return CompressionSpec.Default();
        }

        public string GetName()
        {
            return "HingeSensor" + nameindex.ToString();
        }

        ObservationSpec observationSpec;

        public HingeSensor(MjHingeJoint hinge)
        {
            this.nameindex = i;
            i++;
            this.hinge = hinge;
            // Need to tell the shape so the networks can be configured correctly
            observationSpec = new ObservationSpec(shape: new InplaceArray<int>(3),
                                                  dimensionProperties: new InplaceArray<DimensionProperty>(DimensionProperty.None));
        }

        public ObservationSpec GetObservationSpec()
        {
            return observationSpec;
        }

        public void Reset() { }

        public void Update() { }


    }
    private class VectorSensor : ISensor
    {

        MjSiteVectorSensor imu;
        int nameindex=0;

        

        float[] ContinuousVectorPositions 
        {
            get
            {
                Vector3 reading = imu.SensorReading;
                return new[] {reading.x, reading.y, reading.z};
            }
        }

        

        public List<float> Observations { get => ContinuousVectorPositions.ToList(); } //Combine positions and velocities

        public int Write(ObservationWriter writer)
        {
            writer.AddList(Observations);
            return 3; // Return number of written observations (2 for hinge pos, 1 for hinge vel)
        }

        // Only needed for visual observations (e.g. camera feed)
        public byte[] GetCompressedObservation()
        {
            throw new NotImplementedException();
        }

        public CompressionSpec GetCompressionSpec()
        {
            return CompressionSpec.Default();
        }

        public string GetName()
        {
            return "IMUSensor" + nameindex.ToString();
        }

        ObservationSpec observationSpec;

        public VectorSensor(MjSiteVectorSensor imu)
        {
            this.nameindex = i;
            i++;
            this.imu = imu;
            // Need to tell the shape so the networks can be configured correctly
            observationSpec = new ObservationSpec(shape: new InplaceArray<int>(3),
                                                  dimensionProperties: new InplaceArray<DimensionProperty>(DimensionProperty.None));
        }

        public ObservationSpec GetObservationSpec()
        {
            return observationSpec;
        }

        public void Reset() { }

        public void Update() { }


    }
}
