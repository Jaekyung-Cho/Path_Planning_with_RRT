using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class mazeGenerator : MonoBehaviour
{
    int xLow, xHigh, zLow, zHigh;
    // Use this for initialization
    void Start()
    {
        RRT_star_IncludingReverse RRT = GameObject.Find("Car").GetComponent<RRT_star_IncludingReverse>();
        xLow = (int)RRT.xLow;
        xHigh = (int)RRT.xHigh;
        zLow = (int)RRT.zLow;
        zHigh = (int)RRT.zHigh;

        for (int i = xLow; i < xHigh; i = i + 10)
        {
            for (int j = zLow; j < zHigh; j = j + 10)
            {
                if (Random.Range(0, 100) > 70)
                {
                    if (i != xLow)
                    {
                        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        wall.transform.parent = GameObject.Find("Obstacle").transform;
                        //wall.hideFlags = HideFlags.HideInHierarchy;
                        wall.transform.localScale = new Vector3(0.1f, 4, 10);
                        wall.transform.position = new Vector3(i, 2, j + 5);
                        wall.tag = "obstacle";
                    }
                    if (j != zLow)
                    {
                        GameObject wall2 = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        wall2.transform.parent = GameObject.Find("Obstacle").transform;
                        //wall2.hideFlags = HideFlags.HideInHierarchy;
                        wall2.transform.localScale = new Vector3(10, 4, 0.1f);
                        wall2.transform.position = new Vector3(i + 5, 2, j);
                        wall2.tag = "obstacle";
                    }
                }
            }
        }
    }

    // Update is called once per frame
    void Update()
    {

    }
}
