using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class makeReferencePathSpeed : MonoBehaviour
{

    [HideInInspector] public List<Vector3> referencePath = new List<Vector3>();
    [HideInInspector] public List<bool> IsReverse = new List<bool>();
    public List<float> speed = new List<float>();
    [HideInInspector] public float GoalCost = 0;
    [HideInInspector] public float maxLateralAcceleration = 0.7f * 9.8f;
    [HideInInspector] public float maxspeed, maxspeedRev, accelLimit, deccelLimit;
    // Use this for initialization
    void Start()
    {
        referencePath.Clear();
        IsReverse.Clear();
        speed.Clear();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        RRT_star_IncludingReverse dubins = GameObject.Find("Car").GetComponent<RRT_star_IncludingReverse>();
        Carcontroller car = GameObject.Find("Car").GetComponent<Carcontroller>();
        maxspeed = car.speedLimit;
        maxspeedRev = car.speedLimitRev;
        accelLimit = car.accelerationLimit * 0.7f;
        deccelLimit = car.deccelerationLimit * 0.3f;

        if (dubins.ReferencePath.Count != 0 && GoalCost != dubins.minGoalCost)
        {
            referencePath = dubins.ReferencePath;
            IsReverse = dubins.ReferencePathIsReverse;
            referencePath.Reverse();
            IsReverse.Reverse();
            speed.Clear();
            GoalCost = dubins.minGoalCost;
            speed.Add(0);
            for (int i = 1; i < referencePath.Count - 1; i++)
            {
                if (IsReverse[i] != IsReverse[i - 1] || IsReverse[i + 1] != IsReverse[i])
                {
                    speed.Add(0);
                }
                else
                {
                    float angle = 2 * Vector3.Angle(referencePath[i + 1] - referencePath[i], referencePath[i] - referencePath[i - 1]);
                    if (angle == 0)
                    {
                        if (!IsReverse[i])
                        {
                            speed.Add(maxspeed);
                        }
                        else
                        {
                            speed.Add(maxspeedRev);
                        }
                    }
                    else
                    {
                        float curvature = Mathf.Sqrt(2 - 2 * Mathf.Cos(angle * Mathf.Deg2Rad)) / (referencePath[i + 1] - referencePath[i - 1]).magnitude;
                        if (!IsReverse[i])
                        {
                            speed.Add(Mathf.Min(Mathf.Sqrt(maxLateralAcceleration / curvature), maxspeed));

                        }
                        else
                        {
                            speed.Add(Mathf.Min(Mathf.Sqrt(maxLateralAcceleration / curvature), maxspeedRev));
                        }
                        //Debug.Log(Mathf.Sqrt(maxLateralAcceleration / curvature));
                    }
                }
            }

            speed.Add(0);

            for (int j = 0; j < speed.Count - 1; j++)
            {
                if (speed[j] < speed[j + 1])
                {
                    speed[j + 1] = Mathf.Min(speed[j + 1], Mathf.Sqrt(speed[j] * speed[j] + 2 * accelLimit * (referencePath[j + 1] - referencePath[j]).magnitude));
                }
            }
            for (int k = speed.Count - 1; k > 0; k--)
            {
                if (speed[k - 1] > speed[k])
                {
                    speed[k - 1] = Mathf.Min(speed[k - 1], Mathf.Sqrt(speed[k] * speed[k] - 2 * deccelLimit * (referencePath[k] - referencePath[k - 1]).magnitude));
                }
            }

            Debug.Log("Path updated" + dubins.ReferencePath.Count);
        }
    }
}
