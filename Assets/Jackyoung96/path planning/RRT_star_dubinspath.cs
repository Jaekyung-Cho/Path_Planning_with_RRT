using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RRT_star_dubinspath : MonoBehaviour {
    public float stepSize;
    public float xLow, xHigh, zLow, zHigh;
    public float yLevel, lineWidth;
    public float maxIterations, timestep;
    private Tree theTree;
    private int counter = 0;
    private float timer = 0f;
    public bool enableDrawing = true;
    public bool step = false, disableStepping = false;
    private bool startkey = false;
    private int dimension = 2; // 3차원 드론을 날리면 3으로 바꾸자.(그럴일 없겠지만)
    public float gammaRRT = 140f; //최소값은 2*(1+1/dimension)^(1/dimension) *(area of free space / pi)^(1/dimension) = 138.197
    private Stack<Node> nearX = new Stack<Node> ();
    private Stack<Node> nearX_dup = new Stack<Node> ();
    public float delta = 5f; // collision 방지를 위한 거리
    private float minCost = 1000000f;
    private float minGoalCost = 1000000f;
    private bool startflag = true;
    private float minTurningRadius = 1f;
    private float vectorAngles, theta, alpha, d_f; // dummy value
    private Vector3 center = new Vector3 (0, 0, 0); //dummy value
    private Vector3 startDirection, endDirection;
    private Node finalGoal;

    // Use this for initialization
    void Start () {
        GameObject[] bc = GameObject.FindGameObjectsWithTag ("obstacle");
        for (int i = 0; i < bc.Length; i++) {
            BoxCollider box = bc[i].GetComponent<BoxCollider> ();
            //Debug.Log(box.name);
            Transform trans = bc[i].GetComponent<Transform> ();
            //Debug.Log(trans.lossyScale);
            Vector3 multiplier = new Vector3 (1 + 2 * delta / trans.lossyScale.x, 1, 1 + 2 * delta / trans.lossyScale.z);
            //Debug.Log(multiplier);
            box.size = multiplier;
            //box.size.Set(multiplier.x, multiplier.y, multiplier.z);
            Carcontroller carcontroller = GameObject.Find ("Car").GetComponent<Carcontroller> ();
            float maxSteeringAngle = carcontroller.angleLimit;
            float carLength = carcontroller.L;
            minTurningRadius = carLength / Mathf.Tan (maxSteeringAngle * Mathf.PI / 180); // backwheel minimum radius
            // if you want to use front wheel, use    float minTurningRadius = carLength/Mathf.Sin(maxSteeringAngle*Mathf.pi/180);
        }
    }

    // Update is called once per frame
    void FixedUpdate () // 이건 일정한 시간마다 호출되도록 -> 물리엔진에 적합하다
    {

        if (Input.GetKey (KeyCode.S) && startflag) {
            startkey = true;
            startflag = !startflag;
            moveposition startdir = GameObject.Find ("Car").GetComponent<moveposition> ();
            startDirection = new Vector3(startdir.startDirection,0,0); //수정
            endDirection = new Vector3(startdir.endDirection,0,0); //수정
            finalGoal = new Node (startdir.endPosition);
            finalGoal.faceDirection = endDirection;
            finalGoal.goal = true;
            theTree = new Tree (new Vector3 (transform.position.x, transform.position.y, transform.position.z), startDirection);
            theTree.yL = yLevel;
            theTree.lWidth = lineWidth;
            theTree.enableDrawing = enableDrawing;
            Debug.Log ("start");
            Debug.Log ("start position : " + startdir.startPosition);
            Debug.Log ("start vector direction : " + startDirection);
            Debug.Log ("end position : " + startdir.endPosition);
            Debug.Log ("end vector direction : " + endDirection);
        }
        if (startkey) {
            if (!disableStepping) {
                if (step) // disableStepping 이 false일때, 즉 움직일 수 없을 때 움직임을 시도한다면
                {
                    timer = timestep + 1f;
                    step = false;
                }
            } else {
                timer += Time.deltaTime;
            }
            if (timer > timestep && counter < maxIterations && !theTree.IsComplete ()) {
                
                Node randPoint = new Node(GetRandomPoint());
                //Debug.Log("Random Point" + randPoint.ToString());
                Node nearest = theTree.GetClosestLeaf (ref randPoint, minTurningRadius);
                //Debug.Log("Nearest Point" + nearest.position.ToString());
                if (!nearest.goal) {
                    if (IsColliding (nearest.position, randPoint.position) != -1) {
                        Vector3 direction = randPoint.position - nearest.position;
                        Node newpoint = new Node (nearest.position + direction / direction.magnitude * stepSize);
                        if (IsColliding (nearest.position, newpoint.position) == 1) {
                            newpoint = finalGoal;
                        }
                        float radius = NearRadius (counter, gammaRRT, stepSize, dimension);
                        //Debug.Log(radius);
                        theTree.GetNearLeaf (ref newpoint, radius, ref nearX);
                        Node minX = nearest;
                        minCost = nearest.cost + CostDistance (nearest, ref newpoint, minTurningRadius);
                        newpoint.SetCost (minCost);
                        while (nearX.Count != 0) {
                            Node temp = nearX.Pop ();
                            nearX_dup.Push (temp);
                            if (IsColliding (temp.position, newpoint.position) != -1 && temp.cost + CostDistance (temp, ref newpoint, minTurningRadius) < minCost) {
                                minX = temp;
                                minCost = temp.cost + CostDistance (temp, ref newpoint, minTurningRadius);
                                newpoint.SetCost (minCost);
                            }
                        }
                        while (nearX_dup.Count != 0) {
                            Node temp = nearX_dup.Pop ();
                            if (IsColliding (newpoint.position, temp.position) != -1 && newpoint.cost + CostDistance (newpoint, ref temp, minTurningRadius) < temp.cost) {
                                temp.parent.children.Remove (temp);
                                theTree.AddLeaf (ref newpoint, ref temp);
                                float costChange = -temp.cost + newpoint.cost + CostDistance (newpoint, ref temp, minTurningRadius);
                                temp.SetCost (newpoint.cost + CostDistance (newpoint, ref temp, minTurningRadius));
                                costchange (temp, costChange);
                            }
                        }

                        theTree.AddLeaf (ref minX, ref newpoint);

                        if (IsColliding (minX.position, newpoint.position) == 1) {
                            newpoint = finalGoal;
                            Debug.Log(minGoalCost);
                            Debug.Log(newpoint.cost);
                            if (minGoalCost > newpoint.cost) {
                                GameObject[] deletion = GameObject.FindGameObjectsWithTag ("path");

                                for (int i = 0; i < deletion.Length; i++) {
                                    Destroy (deletion[i]);
                                }
                                minGoalCost = newpoint.cost;
                                theTree.AddFinalNode (ref newpoint);
                                theTree.DrawCompletedPath ();
                                Debug.Log ("final destination : " + newpoint.position);
                                Debug.Log ("Total Cost : " + minGoalCost + " Iteration : " + counter);
                            }
                        }
                        if (theTree.viewfinalNode () != null) {
                            if (theTree.viewfinalNode ().cost < minGoalCost) {
                                GameObject[] deletion = GameObject.FindGameObjectsWithTag ("path");
                                for (int i = 0; i < deletion.Length; i++) {
                                    Destroy (deletion[i]);
                                }
                                minGoalCost = theTree.viewfinalNode ().cost;
                                theTree.DrawCompletedPath ();
                                Debug.Log ("Total Cost : " + minGoalCost + " Iteration : " + counter);
                            }
                        }
                    }
                    counter++;
                    timer = 0;
                }
            }

        }
    }

    public class Node {
        public Vector3 position;
        public Node parent;
        public List<Node> children;
        //public GameObject edgeline;
        public float lWidth = 0.1f;
        public float yL;
        public float cost;
        public Vector3 faceDirection;
        public bool goal, init;

        public Node () {
            parent = null;
            cost = 0;
            children = new List<Node> ();
            position = new Vector3 (0, yL, 0);
            faceDirection = new Vector3 (0, 0, 0);
            goal = false;
            init = false;
            //edgeline = new GameObject();
            //edgeline.tag = "edge";
        }
        public Node (Vector3 x) {
            parent = null;
            cost = 0;
            children = new List<Node> ();
            position = x;
            faceDirection = new Vector3 (0, 0, 0);
            goal = false;
            init = false;
            //edgeline = new GameObject();
            //edgeline.tag = "edge";
        }
        public void AddChild (ref Node x) {
            x.parent = this;
            children.Add (x);
        }
        public void SetParents (Node x) {
            parent = x;
        }
        public void SetCost (float cost) {
            this.cost = cost;
        }
        public void SetFaceDirection (Vector3 x) {
            this.faceDirection = x;
        }
    }

    public class Tree {
        public Node rootNode;
        public float lWidth, yL;
        public Node finalNode;
        private int count = 0;
        private bool complete = false;
        public bool enableDrawing = true;

        public Tree (Vector3 pos, Vector3 face) {
            rootNode = new Node (pos);
            rootNode.faceDirection = face;
            rootNode.init = true;
        }
        public void ViewParent (Node x) //  그냥 디버깅용
        {
            Debug.Log ("Parent is " + x.parent.position);
        }
        public bool IsComplete () {
            return complete;    
        }
        public void AddFinalNode (ref Node k) //path를 찾았을 경우
        {
            finalNode = k;
            //complete = true;
        }
        public Node FindClosestInChildren (Node x, ref Node target, float turnradius) {
            Node closest = x, temp;
            float closestDistance = Vector3.Distance(x.position, target.position);
            float checkDistance = 0f;
            if (x.children.Count != 0) {
                foreach (Node child in x.children) {
                    temp = FindClosestInChildren (child, ref target,turnradius);
                    checkDistance = Vector3.Distance(temp.position, target.position);
                    if (checkDistance < closestDistance) {
                        closestDistance = checkDistance;
                        closest = temp;
                    }
                }
            }
            return closest;
        }
        public Node GetClosestLeaf (ref Node pos, float turnradius) {
            return FindClosestInChildren (rootNode, ref pos, turnradius);
        }
        public Stack<Node> FindNearInChildren (Node x, ref Node target, float radius, ref Stack<Node> stack) {
            Node temp = x;
            float checkDistance = 0f;
            if (temp.children.Count != 0) {
                foreach (Node child in temp.children) {
                    FindNearInChildren (child, ref target, radius, ref stack);
                    checkDistance = Vector3.Distance(child.position, target.position);
                    if (checkDistance < radius) {
                        stack.Push (child);
                    }
                }
            }
            return stack;
        }
        public Stack<Node> GetNearLeaf (ref Node pos, float radius, ref Stack<Node> stack) {
            float checkDistance = 0f;
            checkDistance = Vector3.Distance(rootNode.position, pos.position);
            if (checkDistance < radius) {
                stack.Push (rootNode);
            }
            return FindNearInChildren (rootNode, ref pos, radius, ref stack);
        }

        public void DrawLine (Vector3 _start, Vector3 _end, Color color) {
            Vector3 start = new Vector3 (_start.x, yL, _start.z);
            Vector3 end = new Vector3 (_end.x, yL, _end.z);

            GameObject myLine = new GameObject ();
            if (color == Color.red) {
                myLine.tag = "path";
            }
            if (color == Color.white) {
                myLine.tag = "edge";
            }
            myLine.transform.position = start;
            myLine.AddComponent<LineRenderer> ();
            LineRenderer lr = myLine.GetComponent<LineRenderer> ();
            lr.material = new Material (Shader.Find ("Particles/Alpha Blended Premultiply"));
            lr.startColor = color;
            lr.endColor = color;
            lr.startWidth = lWidth;
            lr.endWidth = lWidth;
            lr.SetPosition (0, start);
            lr.SetPosition (1, end);
        }
        public void DrawCompletedPath (Node x) {
            //Debug.Log("Drawing Completed Recursive");
            /*if (!complete)
            {
                //Debug.Log("Drawing Completed Recursive");
                return;
            }*/
            if (x.parent == null) {
                //Debug.Log("Parent is Null");
                return;
            }
            count++;
            DrawLine (x.parent.position, x.position, Color.red);
            Debug.Log(count+"-th point  position : "+x.position+"direction : "+x.faceDirection);
            DrawCompletedPath (x.parent);
        }
        public void DrawCompletedPath () {
            //Debug.Log("Drawing Completed Non Recursive");
            yL = yL + 0.5f; //선 안곂치게 하기
            count = 0;
            DrawCompletedPath (finalNode);
            yL = yL - 0.5f;
            //Debug.Log("LINES DRAWN:" + count.ToString());
        }

        public Node AddLeaf (ref Node parentLeaf, ref Node childLeaf) {
            parentLeaf.AddChild (ref childLeaf);
            if (enableDrawing) {

                DrawLine (childLeaf.position, parentLeaf.position, Color.white);
            }
            return childLeaf;
        }
/*
        public void CreateAndAdd (Vector3 position) {
            Node nd = new Node (position);
            Node k = GetClosestLeaf (nd.position);
            AddLeaf (ref k, ref nd);
        }*/

        public Node viewfinalNode () {
            return finalNode;
        }
    }

    int IsColliding (Vector3 from, Vector3 to) {
        int temp = 0;
        int layerMask = 1;

        RaycastHit hit;
        Physics.Raycast (from, to - from, out hit, stepSize, layerMask);

        if (hit.collider != null) {
            if (hit.collider.gameObject.tag == "target") {
                temp = 1;
            } else {
                temp = -1;
            }
        }
        return temp;
    }

    Vector3 GetRandomPoint () {
        return new Vector3 (Random.Range (xLow, xHigh), yLevel, Random.Range (zLow, zHigh));
    }

    float NearRadius (int counter, float gamma, float stepsize, int dimension) {
        return Mathf.Min (stepsize, gamma * Mathf.Pow ((Mathf.Log (counter) / counter), 1 / dimension));
    }

    public float CostDistance (Node from, ref Node to, float minRadius) // Dubins path 로 사용
    {
        if (to.goal) {
            return CostForGoal (from, to, minRadius);
        } else {
            vectorAngles = GetAngle (from.faceDirection, to.position - from.position);
            if ((to.position - from.position).magnitude >= 1 - Mathf.Cos (vectorAngles)) {
                if (vectorAngles > 0) {
                    center = from.position + new Vector3 (-from.faceDirection.z, 0, from.faceDirection.x) * minRadius;
                } else {
                    center = from.position + new Vector3 (from.faceDirection.z, 0, -from.faceDirection.x) * minRadius;
                }
                d_f = (to.position - center).magnitude;
                theta = GetAbsAngle (to.position - center, from.position - center) - Mathf.Acos (minRadius / d_f);
                if ((vectorAngles <= 90 && vectorAngles >= 0) || vectorAngles <= -90) {
                    to.faceDirection = RotateVector (from.faceDirection, theta);
                } else {
                    to.faceDirection = RotateVector (from.faceDirection, -theta);
                }
                return Mathf.Sqrt (Mathf.Pow (d_f, 2) - Mathf.Pow (minRadius, 2)) + minRadius * theta;
            } else {
                if (vectorAngles > 0) {
                    center = from.position + new Vector3 (from.faceDirection.z, 0, -from.faceDirection.x) * minRadius;
                } else {
                    center = from.position + new Vector3 (-from.faceDirection.z, 0, from.faceDirection.x) * minRadius;
                }
                d_f = (to.position - center).magnitude;
                theta = Mathf.Asin ((from.faceDirection.x * to.position.x + from.faceDirection.z * to.position.z) / d_f);
                alpha = Mathf.Acos ((5 * Mathf.Pow (minRadius, 2) - Mathf.Pow (d_f, 2)) / 4 * Mathf.Pow (minRadius, 2));
                theta = theta + alpha + Mathf.Asin ((minRadius * Mathf.Sin (alpha)) / d_f);
                if ((vectorAngles <= 90 && vectorAngles >= 0) || vectorAngles <= -90) {
                    to.faceDirection = RotateVector (from.faceDirection, -theta);
                } else {
                    to.faceDirection = RotateVector (from.faceDirection, theta);
                }
                return minRadius * theta;
            }
        }
    }
    public float CostForGoal (Node from, Node to, float minRadius) { // 이거 때매 고장난다...... 각도 같으면 0이라 안됨 고치자 내일
    // from 과 to 의 faceDirection이 모두 고정일 때 거리를 구하는 방법을 모르겠다....
        
        return minRadius * 2 * GetAbsAngle (from.faceDirection, to.faceDirection) * 1000; //For penalty multiplying 10
    }
    void costchange (Node x, float cost) {
        Node temp = x;
        if (temp.children.Count != 0) {
            foreach (Node child in temp.children) {
                costchange (child, cost);
                child.cost = child.cost + cost;
            }
        }
    }
    public float GetAngle (Vector3 from, Vector3 to) //radian
    {
        Vector3 v = from - to;
        return Mathf.Atan2 (v.y, v.x);
    }
    public float GetAbsAngle (Vector3 from, Vector3 to) //radian
    {
        Vector3 v = from / from.magnitude - to / to.magnitude;
        return Mathf.Acos (1 - Mathf.Pow (v.magnitude, 2) / 2);
    }
    public Vector3 RotateVector (Vector3 A, float Yangle) {
        return new Vector3 (A.x * Mathf.Cos (Yangle) - A.z * Mathf.Sin (Yangle), A.y, A.x * Mathf.Sin (Yangle) + A.z * Mathf.Cos (Yangle));
    }
}