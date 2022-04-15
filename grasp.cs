using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;
using UnityEditorInternal;


using System.Collections;
public class grasp : Agent
{
    public GameObject ball;
    public GameObject layer7t;
    //public GameObject[] layer = new GameObject[7];
    GameObject[] layer = new GameObject[8];
    public float[] currentAngle = new float[8];
    public float[] movementAngle = new float[8];
    public float[] target = new float[8];
    public float[] expected = new float[8];
    public bool goal = false;
    public float previousDistance = 100000;
    public float previousDistance_v = 100000;
    public float time_slot = 0.01f;
    public float p_action1 = 0;
    public float p_action2 = 0;
    public float p_action3 = 0;
    public float p_action4 = 0; 
    public float p_action5 = 0; 
    public float p_action6 = 0; 
    public float p_action7 = 0;

    public float p_a1 = 0;
    public float p_a2 = 0;
    public float p_a3 = 0;
    public float p_a4 = 0;
    public float p_a5 = 0;
    public float p_a6 = 0;
    public float p_a7 = 0;

    public float p_aa1 = 0;
    public float p_aa2 = 0;
    public float p_aa3 = 0;
    public float p_aa4 = 0;
    public float p_aa5 = 0;
    public float p_aa6 = 0;
    public float p_aa7 = 0;
    public float Joint_1_max = 2.8973f, Joint_1_min = -2.8973f, Joint_2_max = 1.7628f, Joint_2_min = -1.7628f, Joint_3_max = 2.8973f, Joint_3_min = -2.8973f;
    public float Joint_4_max = -0.0698f, Joint_4_min = -3.0718f, Joint_5_max = 2.8973f, Joint_5_min = -2.8973f, Joint_6_max = 3.7525f, Joint_6_min = -0.0175f;
    public float Joint_7_max = 2.8973f, Joint_7_min = -2.8973f;
    //public float M_PI=3.1415926f;
    public double R_1_max = Math.Sin(2.8973f / Math.PI * 180f / 2f  /180f*Math.PI);
    public double R_2_max = Math.Sin(1.7628f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_3_max = Math.Sin(2.8973f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_4_max = Math.Sin(-0.0698f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_5_max = Math.Sin(2.8973f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_6_max = Math.Sin(3.1f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_7_max = Math.Sin(2.8973f / Math.PI * 180f / 2f / 180f * Math.PI);

    public double R_1_min = Math.Sin(-2.8973f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_2_min = Math.Sin(-1.7628f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_3_min = Math.Sin(-2.8973f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_4_min = Math.Sin(-3.0718f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_5_min = Math.Sin(-2.8973 / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_6_min = Math.Sin(-0.0175f / Math.PI * 180f / 2f / 180f * Math.PI);
    public double R_7_min = Math.Sin(-2.8973f / Math.PI * 180f / 2f / 180f * Math.PI);
    public float time_in=0;
    public Queue E_queue = new Queue();
    public float K_p = 1f, K_i = 0f, K_d = 0f, E_p = 0,E_i=0,E_d=0,E_pid=0;
    public int Q_length = 20;

    public void InitObject()
    {
        string objName;
        for (int i = 0; i < layer.Length; i++)
        {
            objName = string.Format("layer{0}", i);
            layer[i] = GameObject.Find(objName);
        }

    }

    public override void Initialize()
    {
        InitObject();
        
    }
    void Start()
    {
        InitObject();
    }

    public override void OnEpisodeBegin()
    {
        E_queue.Clear();
        E_i = 0;
        for (int i=0;i<Q_length;i++) //should be put in another place
        {
            E_queue.Enqueue(0f);
        }


        ball.transform.position = new Vector3(Random.Range(-2.1f, -4f), Random.Range(0.8f, 4.52f), Random.Range(-2f, 2f));
        previousDistance = Vector3.Distance(layer[7].transform.position, ball.transform.position);
        if (goal)
        {
            ball.transform.position = new Vector3(Random.Range(-2.1f, -4f), Random.Range(0.8f, 4.52f), Random.Range(-2f, 2f));
            goal = false;
        }
        else
        {
            //0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI, M_PI_4
            layer[1].transform.localRotation = Quaternion.Euler(new Vector3(0f, -0f, 0f));//-1 ok
            layer[2].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -45f));//+1
            layer[3].transform.localRotation = Quaternion.Euler(new Vector3(0f, -0f, 0f));//-1
            layer[4].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, 90f));//-1  135
            layer[5].transform.localRotation = Quaternion.Euler(new Vector3(0f, -0f, 0f));//-1
            layer[6].transform.localRotation = Quaternion.Euler(new Vector3(0f, 0f, -45f));//-1   -100
            layer[7].transform.localRotation = Quaternion.Euler(new Vector3(0f, 45f, 0f));//+1
        }
    }




    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 relativePosition = layer[7].transform.position - ball.transform.position;
        float distanceToTarget = Vector3.Distance(ball.transform.position, layer[7].transform.position);
        //oberve delay delay观察
        sensor.AddObservation(layer[1].transform.rotation.y);
        sensor.AddObservation(layer[2].transform.rotation.z);
        sensor.AddObservation(layer[3].transform.rotation.y);
        sensor.AddObservation(layer[4].transform.rotation.z);
        sensor.AddObservation(layer[5].transform.rotation.y);
        sensor.AddObservation(layer[6].transform.rotation.z);
        sensor.AddObservation(layer[7].transform.rotation.y);
        sensor.AddObservation(relativePosition.x);
        sensor.AddObservation(relativePosition.y);
        sensor.AddObservation(relativePosition.z);
        sensor.AddObservation(ball.transform.position);
   //   sensor.AddObservation(v_ball);
        sensor.AddObservation(distanceToTarget);

        //sensor.AddObservation(p_action1);
        //sensor.AddObservation(p_action2);
        //sensor.AddObservation(p_action3);
        //sensor.AddObservation(p_action4);
        //sensor.AddObservation(p_action5);
        //sensor.AddObservation(p_action6);
        //sensor.AddObservation(p_action7);

        sensor.AddObservation(p_a1);
        sensor.AddObservation(p_a2);
        sensor.AddObservation(p_a3);
        sensor.AddObservation(p_a4);
        sensor.AddObservation(p_a5);
        sensor.AddObservation(p_a6);
        sensor.AddObservation(p_a7);


        //   sensor.AddObservation(distanceToTarget_v);
    }

        

    public override void OnActionReceived(float[] vectorAction)
    {
        float distanceToTarget = Vector3.Distance(ball.transform.position, layer[7].transform.position);
         
        float time_scale = 10f;
        var action1 = Mathf.Clamp(vectorAction[0], -1f, 1f) * 0.1246f *  time_scale;
        var action2 = Mathf.Clamp(vectorAction[1], -1f, 1f) * 0.1246f * time_scale;
        var action3 = Mathf.Clamp(vectorAction[2], -1f, 1f) * 0.1246f * time_scale;
        var action4 = Mathf.Clamp(vectorAction[3], -1f, 1f) * 0.1246f * time_scale;
        var action5 = Mathf.Clamp(vectorAction[4], -1f, 1f) * 0.1246f * time_scale;
        var action6 = Mathf.Clamp(vectorAction[5], -1f, 1f) * 0.1246f * time_scale;
        var action7 = Mathf.Clamp(vectorAction[6], -1f, 1f) * 0.1246f * time_scale;


        float a1 = Mathf.Abs((action1 - p_action1) / time_slot);
        float a2 = Mathf.Abs((action2 - p_action2) / time_slot);
        float a3 = Mathf.Abs((action3 - p_action3) / time_slot);
        float a4 = Mathf.Abs((action4 - p_action4) / time_slot);
        float a5 = Mathf.Abs((action5 - p_action5) / time_slot);
        float a6 = Mathf.Abs((action6 - p_action6) / time_slot);
        float a7 = Mathf.Abs((action7 - p_action7) / time_slot);

        float aa1 = Mathf.Abs((a1 - p_a1) / time_slot);
        float aa2 = Mathf.Abs((a2 - p_a2) / time_slot);
        float aa3 = Mathf.Abs((a3 - p_a3) / time_slot);
        float aa4 = Mathf.Abs((a4 - p_a4) / time_slot);
        float aa5 = Mathf.Abs((a5 - p_a5) / time_slot);
        float aa6 = Mathf.Abs((a6 - p_a6) / time_slot);
        float aa7 = Mathf.Abs((a7 - p_a7) / time_slot);







        //限制机械臂各关节角加速度。 
        // (v1-v2)/0.001 = 15/pi*180 = 859.4367
        // (v1-v2)/0.01 = 85.94367


        if (a1 < 859.436* 1 &&
            a2 < 429.718 * 1 &&
            a3 < 572.957 * 1 &&
            a4 < 716.197 * 1 &&
            a5 < 859.436 * 1 &&
            a6 < 1145.9 * 1 &&
            a7 < 1145.9 * 1 &&
            aa1 < 429720 * 1 &&
            aa2 < 214860 * 1 &&
            aa3 < 286480 * 1 &&
            aa4 < 358100 * 1 &&
            aa5 < 429720 * 1 &&
            aa6 < 572960 * 1 &&
            aa7 < 572960 * 1
            )
        {
            layer[1].transform.Rotate(new Vector3(0, 1, 0), action1);  //旋转需要使用四元数 action是度数。
            layer[2].transform.Rotate(new Vector3(0, 0, 1), action2);
            layer[3].transform.Rotate(new Vector3(0, 1, 0), action3);
            layer[4].transform.Rotate(new Vector3(0, 0, 1), action4);
            layer[5].transform.Rotate(new Vector3(0, 1, 0), action5);
            layer[6].transform.Rotate(new Vector3(0, 0, 1), action6);
            layer[7].transform.Rotate(new Vector3(0, 1, 0), action7);

            //p_action1 = action1;
            //p_action2 = action2;
            //p_action3 = action3;
            //p_action4 = action4;
            //p_action5 = action5;
            //p_action6 = action6;
            //p_action7 = action7;

            p_a1 = a1;
            p_a2 = a2;
            p_a3 = a3;
            p_a4 = a4;
            p_a5 = a5;
            p_a6 = a6;
            p_a7 = a7;

            p_aa1 = aa1;
            p_aa2 = aa2;
            p_aa3 = aa3;
            p_aa4 = aa4;
            p_aa5 = aa5;
            p_aa6 = aa6;
            p_aa7 = aa7;
        }
        else
        {
            AddReward(-1.0f);
            EndEpisode();
        }
        //print("Here is ");
        //print(layer[1].transform.localRotation.y);
        //print(layer[2].transform.localRotation.z);
        //print(layer[3].transform.localRotation.y);
        //print(layer[4].transform.localRotation.z);
        //print(layer[5].transform.localRotation.y);
        //print(layer[6].transform.localRotation.z);
        //print(layer[7].transform.localRotation.y);
        if ((layer[6].transform.rotation.z > 0.1305f) || (layer[6].transform.rotation.z < -0.1305f) ||
           (layer[6].transform.rotation.x > 0.1305f) || (layer[6].transform.rotation.x < -0.1305f))
        {
            AddReward(-1f);
            EndEpisode();
        }
        if ((layer[7].transform.position.y) > 20f)
        {
            //(layer[7].transform.position.y < ball.transform.position.y)
            AddReward(-100f);
            EndEpisode();
        }
        //限制机械臂各关节转动范围.rotation 是用四元数表示。角度在代码里是（0~360）度打印出来。四元数是角度的直接计算。
        if ((layer[1].transform.localRotation.y) < -R_1_max ||
                   (layer[1].transform.localRotation.y) > -R_1_min ||
                   (layer[2].transform.localRotation.z) <  R_2_min ||
                   (layer[2].transform.localRotation.z) >  R_2_max ||
                   (layer[3].transform.localRotation.y) < -R_3_max ||
                   (layer[3].transform.localRotation.y) > -R_3_min ||
                   (layer[4].transform.localRotation.z) < -R_4_max ||
                   (layer[4].transform.localRotation.z) > -R_4_min ||
                   (layer[5].transform.localRotation.y) < -R_5_max ||
                   (layer[5].transform.localRotation.y) > -R_5_min ||
                   (layer[6].transform.localRotation.z) < -R_6_max ||
                   (layer[6].transform.localRotation.z) > -R_6_min ||
                   (layer[7].transform.localRotation.y) <  R_7_min ||
                   (layer[7].transform.localRotation.y) >  R_7_max)

        {
          AddReward(-1.0f);
          EndEpisode();
        }

        //       if (Mathf.Abs((action1 - p_action1) / time_slot) > 85.9436    ||
        //           Mathf.Abs((action2 - p_action2) / time_slot) > 42.9718    ||
        //           Mathf.Abs((action3 - p_action3) / time_slot) > 57.2957    ||
        //           Mathf.Abs((action4 - p_action4) / time_slot) > 71.6197    ||
        //           Mathf.Abs((action5 - p_action5) / time_slot) > 85.9436    ||
        //           Mathf.Abs((action6 - p_action6) / time_slot) > 114.59     ||
        //           Mathf.Abs((action7 - p_action7) / time_slot) > 114.59)
        //        {
        //            AddReward(-5.0f);
        //            EndEpisode();
        //        }

        E_p = distanceToTarget;
        E_d = (distanceToTarget - previousDistance);
        float E_temp = (float)E_queue.Dequeue();
        E_queue.Enqueue(E_p);
        E_i = E_i + (E_p - E_temp) / Q_length;

        E_pid = K_p * E_p + K_i * E_i + K_d * E_d;


        if (distanceToTarget < 0.1f)
        {
            // goal = true;
            //AddReward(5.0f);
            //EndEpisode();
            time_in++;
        }
        else
        {
            time_in = 0;

        }



        if (time_in>100)
        {
            AddReward(100.0f);
            EndEpisode();
        }
      
        AddReward((float)Math.Exp(-E_pid));


        //if ((distanceToTarget < previousDistance)|| time_in>0)
        //{
        //   AddReward(1f);
        //}
        //else
        //{
        //   AddReward(-1f);
        //}





        previousDistance = distanceToTarget;
        //previousDistance_v = distanceToTarget_v;
        //p_layer_6 = layer[6].transform.eulerAngles.z;



    }
    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = Input.GetAxis("Horizontal");
        actionsOut[1] = Input.GetAxis("Vertical");
    }
}

