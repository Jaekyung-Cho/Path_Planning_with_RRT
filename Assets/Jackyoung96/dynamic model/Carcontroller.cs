using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Carcontroller : MonoBehaviour
{
    public float speed = 0;
    public float Acceleration = 0;
    public float speedLimit = 10;
    public float speedLimitRev = 3;
    public float accelerationLimit = 1.8f;
    public float deccelerationLimit = -6;
    [HideInInspector] public float RateOfAccelLimit = 6, RateOfDeccelLimit = -20;
    public float autodecceleration = -3;

    public float angle = 0;
    public float angleLimit = 30;
    public float velAngleLimit = 0.3294f * Mathf.Rad2Deg;
    public float autoangleDecceleration = 0;

    [HideInInspector] public bool ReverseGear = false;
    public float rotation = 0;
    public float L = 2.8F; // 앞뒤 바퀴 사이 거리
    [HideInInspector] public float minTurningRadius;


    Rigidbody rid;
    Rigidbody rightwheel;
    Rigidbody leftwheel;

    // Use this for initialization
    void Start()
    {
        rid = GameObject.Find("Car").GetComponent<Rigidbody>();
        rightwheel = GameObject.FindWithTag("leftwheel").GetComponent<Rigidbody>();
        leftwheel = GameObject.FindWithTag("rightwheel").GetComponent<Rigidbody>();
        minTurningRadius = L / Mathf.Tan(angleLimit * Mathf.PI / 180);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        MoveCar();
    }
    private void MoveCar()
    {
        PID pid = GameObject.Find("Car").GetComponent<PID>();
        float RateOfAccel = (float)pid.RateOfAccel;
        float angularvel = (float)pid.angularvel;
        float temp_angle = angle;
        ReverseGear = pid.ReverseGear;
        if(speed == 0){
            Acceleration = 0;
            
        }

        if (angularvel > velAngleLimit)
        {
            angularvel = velAngleLimit;
        }
        else if (angularvel < -velAngleLimit)
        {
            angularvel = -velAngleLimit;
        }

        /*if (RateOfAccel > RateOfAccelLimit)
        {
            RateOfAccel = RateOfAccelLimit;
        }
        else if (RateOfAccel < RateOfDeccelLimit)
        {
            RateOfAccel = RateOfDeccelLimit;
        }*/

        if (RateOfAccel > 0)
        {
            RateOfAccel = RateOfAccelLimit;
        }
        else if (RateOfAccel < 0)
        {
            RateOfAccel = RateOfDeccelLimit;
        }
        else{
            RateOfAccel = 0;
        }


        Acceleration += RateOfAccel * Time.deltaTime;

        if (Acceleration > accelerationLimit)
        {
            Acceleration = accelerationLimit;
        }
        else if (Acceleration < deccelerationLimit)
        {
            Acceleration = deccelerationLimit;
        }


        if (!ReverseGear)
        {
            speed += Acceleration * Time.deltaTime;
            if (speed > speedLimit) speed = speedLimit;
            if (speed < 0) speed = 0;
        }
        else
        {
            speed -= Acceleration * Time.deltaTime;
            if (speed > 0) speed = 0;
            if (speed < -speedLimitRev) speed = -speedLimitRev;
        }

        //if (speed < 0.5 && speed > -0.5) speed = 0;

        /*
                if (Acceleration > 0)
                {
                    if (!ReverseGear)
                    {
                        speed += Acceleration * Time.deltaTime;
                        if (speed > speedLimit) speed = speedLimit;
                        if (speed < -speedLimit) speed = -speedLimit;
                    }
                    else
                    {
                        speed -= Acceleration * Time.deltaTime;
                        if (speed > speedLimitRev) speed = speedLimitRev;
                        if (speed < -speedLimitRev) speed = -speedLimitRev;
                    }
                }
                else if (Acceleration < 0)
                {
                    if (Mathf.Abs(speed) <= speedLimit)
                    {
                        if (!ReverseGear)
                        {
                            speed -= deccelerationLimit * Acceleration * Time.deltaTime;
                        }
                        else
                        {
                            speed += deccelerationLimit * Acceleration * Time.deltaTime;
                        }
                        if (speed > speedLimit) speed = speedLimit;
                        if (speed < -speedLimit) speed = -speedLimit;
                    }
                }
                else
                {
                    if (speed > 0.5)
                        speed += autodecceleration * Time.deltaTime;
                    else if (speed < -0.5)
                        speed -= autodecceleration * Time.deltaTime;
                    else
                        speed = 0;
                }*/



        angle += angularvel * Time.deltaTime;
        if (angle > angleLimit) angle = angleLimit;
        if (angle < -angleLimit) angle = -angleLimit;
        //if (angle < 0.5 && angle > -0.5) angle = 0;

        rotation = speed * Mathf.Tan(angle * Mathf.PI / 180) / L;

        Vector3 wheelangle = new Vector3(0f, 1, 0f) * (angle - temp_angle);
        Vector3 velocity = transform.right * speed;
        Vector3 Rotation = new Vector3(0f, 1, 0f) * rotation * 180 / Mathf.PI;

        rid.MovePosition(transform.position + velocity * Time.deltaTime);
        rid.MoveRotation(rid.rotation * Quaternion.Euler(Rotation * Time.deltaTime));
        rightwheel.MoveRotation(rightwheel.rotation * Quaternion.Euler(wheelangle + Rotation * Time.deltaTime));
        leftwheel.MoveRotation(leftwheel.rotation * Quaternion.Euler(wheelangle + Rotation * Time.deltaTime));
    }

}
