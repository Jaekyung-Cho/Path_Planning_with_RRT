using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class moveposition : MonoBehaviour {

    GameObject temp;
    Transform tr;
    
    public float startDirection = 0;
    public float endDirection = 0;

    public Vector3 startPosition;
    public Vector3 endPosition;
    public float rotationVelocity = 3;
    public float transformVelocity = 10;
	// Use this for initialization
	void Start () {
		startPosition = GameObject.Find("Car").transform.position;
        endPosition = GameObject.Find("target").transform.position;
        startDirection = GameObject.Find("Car").transform.rotation.eulerAngles.y * Mathf.Deg2Rad;
        endDirection = GameObject.Find("target").transform.rotation.eulerAngles.y * Mathf.Deg2Rad;
	}
	
	// Update is called once per frame
	void FixedUpdate () {
        if (Input.GetKey(KeyCode.T))
        {
            temp = GameObject.Find("target");
            tr = temp.transform;
            if (Input.GetKey(KeyCode.LeftArrow))
            {
                Vector3 move = new Vector3(tr.position.x - 1, tr.position.y, tr.position.z);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if (Input.GetKey(KeyCode.RightArrow))
            {
                Vector3 move = new Vector3(tr.position.x + 1, tr.position.y, tr.position.z);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if (Input.GetKey(KeyCode.UpArrow))
            {
                Vector3 move = new Vector3(tr.position.x , tr.position.y, tr.position.z + 1);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if(Input.GetKey(KeyCode.DownArrow))
            {
                Vector3 move = new Vector3(tr.position.x, tr.position.y, tr.position.z - 1);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if(Input.GetKey(KeyCode.Q))
            {
                tr.Rotate(0,-rotationVelocity,0);
            }
            if(Input.GetKey(KeyCode.E))
            {
                tr.Rotate(0,rotationVelocity,0);
            }
            endDirection = -tr.eulerAngles.y * Mathf.Deg2Rad;
            endPosition = tr.position;
        }
        if (Input.GetKey(KeyCode.C))
        {
            temp = GameObject.Find("Car");
            tr = temp.transform;
            if (Input.GetKey(KeyCode.LeftArrow))
            {
                Vector3 move = new Vector3(tr.position.x - 1, tr.position.y, tr.position.z);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if (Input.GetKey(KeyCode.RightArrow))
            {
                Vector3 move = new Vector3(tr.position.x + 1, tr.position.y, tr.position.z);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if (Input.GetKey(KeyCode.UpArrow))
            {
                Vector3 move = new Vector3(tr.position.x, tr.position.y, tr.position.z + 1);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if (Input.GetKey(KeyCode.DownArrow))
            {
                Vector3 move = new Vector3(tr.position.x, tr.position.y, tr.position.z - 1);
                tr.position = Vector3.MoveTowards(tr.position, move, Time.deltaTime * transformVelocity);
            }
            if(Input.GetKey(KeyCode.Q))
            {
                tr.Rotate(0,-rotationVelocity,0);
            }
            if(Input.GetKey(KeyCode.E))
            {
                tr.Rotate(0,rotationVelocity,0);
            }
            startDirection = -tr.eulerAngles.y * Mathf.Deg2Rad;
            startPosition = tr.position;
        }
	}

    

    public Vector3 RotateVector (Vector3 A, float Yangle) {
        return new Vector3 (A.x * Mathf.Cos (Yangle) - A.z * Mathf.Sin (Yangle), A.y, A.x * Mathf.Sin (Yangle) + A.z * Mathf.Cos (Yangle));
    }
}
