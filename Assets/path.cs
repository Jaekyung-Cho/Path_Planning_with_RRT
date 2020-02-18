using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class path : MonoBehaviour
{

    Vector3 temp = new Vector3(0, 0, 0);
	bool flag = true;

    // Use this for initialization
    void Start()
    {
        Transform pos = GameObject.Find("Car").GetComponent<Transform>();
        temp = pos.position;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        PID pid = GameObject.Find("Car").GetComponent<PID>();

        if (pid.startflag)
        {
            Transform pos = GameObject.Find("Car").GetComponent<Transform>();
			if(flag){
				temp = pos.position;
				flag = false;
			}
            Vector3 now = pos.position;

            if ((now - temp).magnitude > 1)
            {
                drawline(temp, now, Color.yellow);

                temp = now;
            }
        }
    }
    static void drawline(Vector3 from, Vector3 to, Color color)
    {
        GameObject line = new GameObject();
        line.hideFlags = HideFlags.HideInHierarchy;
        line.AddComponent<LineRenderer>();
        line.tag = "realpath";
        LineRenderer lr = line.GetComponent<LineRenderer>();
        lr.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
        lr.startColor = color;
        lr.endColor = color;
        lr.startWidth = 0.2f;
        lr.endWidth = 0.2f;
        lr.SetPosition(0, new Vector3(from.x, 0.5f, from.z));
        lr.SetPosition(1, new Vector3(to.x, 0.5f, to.z));
    }
}
