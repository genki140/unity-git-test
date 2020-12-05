using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Unity.Barracuda;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class WalkRobotAgent : Agent
{
    public ConfigurableJoint[] Joints = new ConfigurableJoint[0];
    public GameObject Camera = null;

    public Unity.Barracuda.NNModel model;

    private Rigidbody Rigid;

    private class RigidTransform
    {
        public Rigidbody Rigid;
        public Vector3 Position;
        public Quaternion Rotation;
    }

    private List<RigidTransform> KeepRigidTransforms;

    private List<CollisionSensor> CollisionSensors;


    Dictionary<string, float> taggedReward;

    public void AddReward(float value, string tag)
    {
        AddReward(value);
        if (taggedReward.ContainsKey(tag) == false) { taggedReward[tag] = 0; }
        taggedReward[tag] += value;
    }


    public override void Initialize()
    {
        Rigid = GetComponent<Rigidbody>();
        KeepRigidTransforms = GetComponentsInChildren<Rigidbody>().Concat(new[] { Rigid }).Select(x => new RigidTransform
        {
            Rigid = x,
            Position = x.position,
            Rotation = x.rotation,
        }).ToList();

        CollisionSensors = GetComponentsInChildren<CollisionSensor>().ToList();
    }

    public override void OnEpisodeBegin()
    {
        foreach (var KeepRigidTransform in KeepRigidTransforms)
        {
            KeepRigidTransform.Rigid.rotation = KeepRigidTransform.Rotation;
            KeepRigidTransform.Rigid.position = KeepRigidTransform.Position;
            KeepRigidTransform.Rigid.angularVelocity = Vector3.zero;
            KeepRigidTransform.Rigid.velocity = Vector3.zero;
        }

        if (taggedReward == null)
        {
            taggedReward = new Dictionary<string, float>();
        }
        else
        {
            foreach (var tag in taggedReward)
            {
                Debug.Log("Reward[" + tag.Key + "]=" + tag.Value.ToString());
            }
            taggedReward.Clear();
        }
    }

    public override void CollectObservations(VectorSensor sensor) // ( 10 / 秒 )
    {
        // 10/s　だと、接地時の sumChangeVelocity.y は、0.98くらいになる。

        //自分空間の速度情報を与える
        var directionVector = Rigid.transform.InverseTransformDirection(Rigid.velocity);
        sensor.AddObservation(directionVector.normalized);
        sensor.AddObservation(directionVector.magnitude);

        //自分の加速度情報を与える（重力方向含む）
        var directionVectorChange = Rigid.transform.InverseTransformDirection(sumChangeVelocity);
        sensor.AddObservation(directionVectorChange.normalized);
        sensor.AddObservation(directionVectorChange.magnitude);

        //自分空間の回転角を与える
        var angularVelocity = Rigid.transform.InverseTransformDirection(Rigid.angularVelocity);
        sensor.AddObservation(angularVelocity.normalized);
        sensor.AddObservation(angularVelocity.magnitude);


        //--------------報酬計算--------------

        //速度変化が大きすぎると疲れるし、下が重力でないと疲れる。(等加速度で接地しているのが一番)
        var dirSubMag = (directionVectorChange - (gravityVector * 0.1f)).magnitude;
        dirSubMag = System.Math.Max(dirSubMag - 2.0f, 0);
        AddReward(dirSubMag * -0.03f, "VelChange");

        //足が感じる衝撃をセンサーとダメージに計算
        foreach (var x in CollisionSensors)
        {
            sensor.AddObservation(x.SumImpulseMagnitude);

            //リセット
            x.SumImpulseMagnitude = 0;
        }

        //リセット
        sumChangeVelocity = Vector3.zero;
    }


    public override void OnActionReceived(float[] vectorAction)
    {
        float angleMax = 45.0f;
        float forceMax = 500.0f;
        float damper = 0.0f;

        float forces = 0;

        int index = 0;
        foreach (var item in Joints.Select((joint, i) => new { joint, i }))
        {
            if (item.i % 2 == 0)
            {
                var y = vectorAction[index] * angleMax;
                index++;
                var z = vectorAction[index] * angleMax;
                index++;
                var x = vectorAction[index] * angleMax;
                index++;
                item.joint.targetRotation = Quaternion.Euler(x, y, z);

                var f = (vectorAction[index] * 0.5f + 0.5f);
                forces += f;
                f *= forceMax;
                index++;
                item.joint.angularXDrive = new JointDrive()
                {
                    maximumForce = float.MaxValue,
                    positionSpring = f,
                    positionDamper = damper,
                };
                item.joint.angularYZDrive = new JointDrive()
                {
                    maximumForce = float.MaxValue,
                    positionSpring = f,
                    positionDamper = damper,
                };
            }
            else
            {
                var z = vectorAction[index] * angleMax;
                index++;
                item.joint.targetRotation = Quaternion.Euler(0, 0, z);

                var f = (vectorAction[index] * 0.5f + 0.5f);
                forces += f;
                f *= forceMax;
                index++;
                item.joint.angularYZDrive = new JointDrive()
                {
                    maximumForce = float.MaxValue,
                    positionSpring = f,
                    positionDamper = damper,
                };
            }
        }

        //Debug.Log(forces);
        AddReward(System.Math.Max(forces - 2, 0) * -0.0002f, "forces"); //力を多く使うと疲れる。


        //高さを保っていれば報酬。(1.3の高さが最大)
        var heightReward = Mathf.Min(Rigid.transform.position.y - 0.3f, 1) * 0.01f;
        AddReward(heightReward, "height");
        //Debug.Log("height:" + heightReward);


        //スピードが出るほど報酬。
        var directionReward = Rigid.transform.InverseTransformDirection(Rigid.velocity).z * 0.003f;
        AddReward(directionReward, "speed");





        //足が感じる衝撃をダメージに計算
        foreach (var x in CollisionSensors)
        {
            AddReward(System.Math.Max(x.SumImpulseMagnitudeForAction - 3.0f, 0) * -0.01f, "impuls");

            //リセット
            x.SumImpulseMagnitudeForAction = 0;
        }



    }



    public override void Heuristic(float[] actionsOut)
    {
        var index = 0;
        foreach (var n in Enumerable.Range(0, Joints.Length / 2))
        {
            actionsOut[index] = 0;
            index++;
            actionsOut[index] = 0;
            index++;
            actionsOut[index] = 0;//Input.GetAxis("Vertical") == 0 ? 0 : (Input.GetAxis("Vertical") > 0 ? 1 : -1);
            index++;

            actionsOut[index] = Input.GetAxis("Vertical"); //arm spring
            index++;

            actionsOut[index] = 0;
            index++;

            actionsOut[index] = Input.GetAxis("Horizontal"); //hand spring
            index++;

            //actionsOut[1] = Input.GetAxis("Vertical");


            // actionsOut[index] = Random.value * 2 - 1;
            // index++;
            // actionsOut[index] = Random.value * 2 - 1;
            // index++;
            // actionsOut[index] = Random.value * 2 - 1;
            // index++;
            // actionsOut[index] = Random.value * 2 - 1;
            // index++;

        }
    }

    //加速度計算。
    Vector3 previousVelocity; //global
    Vector3 sumChangeVelocity;//global
    Vector3 gravityVector = new Vector3(0, 9.80665f, 0);  // m/s2



    void Update() //呼ばれる頻度不定
    {
        var changeVelocity = (Rigid.velocity - previousVelocity);
        var gravityDelta = gravityVector * Time.deltaTime;
        sumChangeVelocity += changeVelocity + gravityDelta;
        previousVelocity = Rigid.velocity;

        //Debug.Log("vel:" + changeVelocity);
        //Debug.Log("gra:" + gravityDelta);

        if (Input.GetKeyDown(KeyCode.B))
        {
            Debug.Log("ChangeBrain");

            //ブレインをファイルから設定してみる。 
            var filePath = @"C:\Users\genki\Documents\source\unity\test.bin";
            var bytes = System.IO.File.ReadAllBytes(filePath);

            var model = ScriptableObject.CreateInstance<Unity.Barracuda.NNModel>();
            model.name = "WrlkRobot-5999936";
            model.hideFlags = HideFlags.NotEditable;
            model.modelData = ScriptableObject.CreateInstance<Unity.Barracuda.NNModelData>();
            model.modelData.name = "Data";
            model.modelData.Value = bytes;
            model.modelData.hideFlags = HideFlags.NotEditable;

            // Debug.Log("Model.name = " + model.name);
            // Debug.Log("Model.hideFlags = " + model.hideFlags);
            // Debug.Log("Model.modelData.name = " + model.modelData.name);
            // Debug.Log("Model.modelData.hideFlags = " + model.modelData.hideFlags);
            // Debug.Log("Model.modelData.Value.Length = " + model.modelData.Value.Length); //2389340

            // //ファイルサイズは2388279なので、何か追加している。


            //var outbinPath = @"C:\Users\genki\Documents\source\unity\test.bin";
            //File.WriteAllBytes(outbinPath, model.modelData.Value);

            SetModel("WalkRobot", model);
        }


        if (Camera)
        {
            Camera.transform.localPosition = Rigid.transform.localPosition; //new Vector3(Rigid.transform.localPosition.x, 0, Rigid.transform.localPosition.z);
            Camera.transform.localEulerAngles = new Vector3(0, Rigid.transform.localEulerAngles.y, 0);
        }
    }


    void Test()
    {
        // var importer = new Unity.Barracuda.NNModelImporter();
        // //importer.

        // //.ModelLoader.Load("file");

        // SetModel("WalkRobot", Unity.Barracuda.NNModel );


        // var mmm = new Unity.Barracuda.Model();

        // //GetComponent<Unity.MLAgents.Agent>

        // Unity.Barracuda.NNModel.CreateInstance;

        // Unity.Barracuda.WorkerFactory.CreateWorker
        // var test = new Unity.Barracuda.NNModel();
    }

}
