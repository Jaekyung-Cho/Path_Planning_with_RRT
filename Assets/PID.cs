using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PID : MonoBehaviour
{
    [HideInInspector] public float totalCost;
    [HideInInspector] public float speed = 0;
    [HideInInspector] public float speedLimit = 10, speedLimitRev = 3;
    [HideInInspector] public float accelerationLimit = 6.77F;
    [HideInInspector] public float deccelerationLimit = 3.75F;

    [HideInInspector] public float angle = 0;
    [HideInInspector] public float angleLimit = 30;
    [HideInInspector] public float angleaccelerationRate = 45;
    [HideInInspector] public float angleDecceleration = 45;

    [HideInInspector] public float rotation = 0;
    [HideInInspector] public float L = 2.8F; // 앞뒤 바퀴 사이 거리		
    [HideInInspector] public float minTurningRadius = 0;
    [HideInInspector] public bool startflag = false;
    [HideInInspector] public bool ReverseGear = false;
    private int numForDistance = 1, numForNearest = 1;
    [HideInInspector] public List<Vector3> ReferencePath = new List<Vector3>();
    [HideInInspector] public List<bool> IsReverse = new List<bool>();
    [HideInInspector] public List<float> ReferenceSpeed = new List<float>();
    [HideInInspector] public float RateOfAccel = 0;
    [HideInInspector] public float angularvel = 0;
    //public float K_v = 10;
    //public float K_s = 1;
    public float K_a = 500;
    public float K_p = 1;
    public float K_pp = 1;
    [HideInInspector] public bool brakeflag = false;
    public float pathSmoothConst = 1;
    public float safetyFactor = 2;





    // Use this for initialization
    void Start()
    {
        ReferencePath.Clear();
        Carcontroller car = GameObject.Find("Car").GetComponent<Carcontroller>();
        speedLimit = car.speedLimit;
        speedLimitRev = car.speedLimitRev;
        accelerationLimit = car.accelerationLimit;
        deccelerationLimit = car.deccelerationLimit;
        angleLimit = car.angleLimit;
        angleDecceleration = car.autoangleDecceleration;
        L = car.L;
        minTurningRadius = pathSmoothConst * L / Mathf.Tan(angleLimit * Mathf.Deg2Rad);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Input.GetKey(KeyCode.M) && !startflag)
        {
            GameObject[] bc = GameObject.FindGameObjectsWithTag("obstacle");
            for (int i = 0; i < bc.Length; i++)
            {
                BoxCollider box = bc[i].GetComponent<BoxCollider>();
                box.size = new Vector3(1,1,1);
            }

            startflag = true;
            makeReferencePathSpeed path = GameObject.Find("Car").GetComponent<makeReferencePathSpeed>();
            ReferencePath = path.referencePath;
            IsReverse = path.IsReverse;
            ReferenceSpeed = path.speed;
            /*for (int j = 0; j < ReferencePath.Count; j++)
            {
                Debug.Log(ReferencePath[j]);
            }*/
            numForDistance = 1;
        }
        if (startflag)
        {
            Transform pos = GameObject.Find("Car").GetComponent<Transform>();
            Vector3 presentPos = new Vector3(pos.position.x, 0, pos.position.z);
            Carcontroller car = GameObject.Find("Car").GetComponent<Carcontroller>();
            speed = car.speed;
            angle = - car.angle;

            //For LookingForward distance
            if (numForDistance < ReferencePath.Count)
            {
                while ((ReferencePath[numForDistance] - presentPos).magnitude < LFdistance(speed, angle) && IsReverse[numForDistance]==IsReverse[numForDistance-1])
                {
                    if(IsReverse[numForDistance]!=IsReverse[numForDistance-1]){
                        Debug.Log("Gear Changed");
                    }
                    numForDistance++;
                    ReverseGear = IsReverse[numForDistance-1];
                    if(numForDistance >= ReferencePath.Count -1){
                        numForDistance = ReferencePath.Count - 1;
                        break;
                    }
                    
                }
            }

            while(numForNearest <= numForDistance && numForNearest < ReferencePath.Count - 1 && (ReferencePath[numForNearest]-presentPos).magnitude >= (ReferencePath[numForNearest+1]-presentPos).magnitude){           
                numForNearest++;
            }

            Vector3 direction = ReferencePath[numForDistance] - presentPos;
            drawline(presentPos, presentPos + direction, Color.red);

            // steering  P  control ( Pure Pursuit )
            float theta = (360 - pos.eulerAngles.y) * Mathf.Deg2Rad;
            if (theta > Mathf.PI)
            {
                theta -= 2 * Mathf.PI;
            }
            theta = Mathf.Atan2(direction.z, direction.x) - theta;
            if(theta < -Mathf.PI){
                theta += 2*Mathf.PI;
            }else if(theta > Mathf.PI){
                theta -= 2* Mathf.PI;
            }

            float error = Mathf.Atan(2*(L/direction.magnitude)*Mathf.Sin(theta)) - angle * Mathf.Deg2Rad;
            angularvel = - K_a * error;


            // velocity P control
            RateOfAccel = K_pp * (-K_p * (Mathf.Abs(speed) - ReferenceSpeed[numForNearest])-car.Acceleration);
            Debug.Log(RateOfAccel);
            if(numForNearest < ReferencePath.Count - 2){
                if((speed <0.01 || speed > -0.01) && IsReverse[numForNearest +1 ] != IsReverse[numForNearest]){
                    while(ReferenceSpeed[numForNearest]==0){
                        numForNearest++;
                        numForDistance = numForNearest;
                    }
                }
            }
            else{
                if((speed <0.01 || speed > -0.01)){
                    startflag = false;
                    Debug.Log("end");
                }
            }
        }
    }

    public void drawline(Vector3 from, Vector3 to, Color color){
        GameObject line = new GameObject();
        line.hideFlags = HideFlags.HideInHierarchy;
        line.AddComponent<LineRenderer>();
        line.tag = "laser";
        LineRenderer lr = GameObject.FindWithTag("laser").GetComponent<LineRenderer>();
        lr.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
        lr.startColor = color;
        lr.endColor = color;
        lr.startWidth = 1f;
        lr.endWidth = 1f;
        lr.SetPosition(0, from);
        lr.SetPosition(1, to);
    }

    //Looking Forward Distance
    public static float LFdistance(float vel, float angle)
    {
        if(vel >= 0){
            //if(angle == 30 || angle == -30) return 1.5f;
            if(vel < 1.34) return 3;
            else if(vel < 5.36) return 2.24f * vel;
            else return 12;
        }
        else{
            if(vel > -1.34/10*3) return 3/10*3*2;
            else if(vel > -5.36/10*3) return -2.24f/10*3 *2* vel;
            else return 12/10*3*2;
        }
    }

    public static float safeDistance(float vel, float maxAccel, float safetyFactor){
        return - safetyFactor *  vel * vel / (2 * maxAccel) + 0.5f;
    }
}
