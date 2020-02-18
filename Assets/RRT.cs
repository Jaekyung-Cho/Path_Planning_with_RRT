using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RRT : MonoBehaviour {
    public float stepSize;
    public float xLow, xHigh, zLow, zHigh;
    public float yLevel, lineWidth;
    public float maxIterations, timestep;
    private Tree theTree;
    private int counter = 0;
    private float timer = 0f;
    public bool enableDrawing = true;
    public bool step = false, disableStepping = false;
    public bool startkey = false;

    // Use this for initialization
    void Start() {
        /*theTree = new Tree(new Vector3(transform.position.x, transform.position.y, transform.position.z));
        theTree.yL = yLevel;
        theTree.lWidth = lineWidth;
        theTree.enableDrawing = enableDrawing;
        Debug.Log("start");*/
    }

    // Update is called once per frame
    void FixedUpdate() // 이건 일정한 시간마다 호출되도록 -> 물리엔진에 적합하다
    {
        if (Input.GetKey(KeyCode.S))
        {
            startkey = true;
            theTree = new Tree(new Vector3(transform.position.x, transform.position.y, transform.position.z));
            theTree.yL = yLevel;
            theTree.lWidth = lineWidth;
            theTree.enableDrawing = enableDrawing;
            Debug.Log("start");
        }
        if (startkey)
        {
            if (!disableStepping)
            {
                if (step) // disableStepping 이 false일때, 즉 움직일 수 없을 때 움직임을 시도한다면
                {
                    timer = timestep + 1f;
                    step = false;
                }
            }
            else
            {
                timer += Time.deltaTime;
            }
            if (timer > timestep && counter < maxIterations && !theTree.IsComplete())
            {
                Vector3 randPoint = GetRandomPoint();
                //Debug.Log("Random Point" + randPoint.ToString());
                Node nearest = theTree.GetClosestLeaf(randPoint);
                //Debug.Log("Nearest Point" + nearest.position.ToString());
                if (IsColliding(nearest.position, randPoint) != -1)
                {
                    Vector3 direction = randPoint - nearest.position;
                    Node newpoint = new Node(nearest.position + direction / direction.magnitude * stepSize);
                    theTree.AddLeaf(ref nearest, ref newpoint);
                    if (IsColliding(nearest.position, randPoint) == 1)
                    {
                        Debug.Log("Complete");
                        theTree.AddFinalNode(ref newpoint);
                        theTree.DrawCompletedPath();
                        Debug.Log("Total count:" + counter);
                    }
                }
                counter++;
                timer = 0;
            }
        }
    }


    public class Node{
        public Vector3 position;
        public Node parent;
        public List<Node> children;
        public GameObject edgeline;
        public float lWidth = 0.1f;
        public float yL;
        
        public Node()
        {
            parent = null;
            children = new List<Node>();
            position = new Vector3(0,yL,0);
            edgeline = new GameObject();
            edgeline.tag = "edge";
        }
        public Node(Vector3 x)
        {
            parent = null;
            children = new List<Node>();
            position = x;
            edgeline = new GameObject();
            edgeline.tag = "edge";
        }
        public void AddChild(ref Node x)
        {
            x.parent = this;
            children.Add(x);
        }
        public void SetParents(Node x)
        {
            parent = x;
        }
        public void DrawLine(Color color)
        {
            if(parent == null)
            {
                return;
            }
            edgeline = new GameObject();
            edgeline.tag = "edge";
            Vector3 start = new Vector3(parent.position.x, yL, parent.position.z);
            Vector3 end = new Vector3(position.x, yL, position.z);

            edgeline.transform.position = start;
            edgeline.AddComponent<LineRenderer>();
            LineRenderer lr = edgeline.GetComponent<LineRenderer>();
            lr.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
            lr.startColor = color;
            lr.endColor = color;
            lr.startWidth = lWidth;
            lr.endWidth = lWidth;
            lr.SetPosition(0, start);
            lr.SetPosition(1, end);
        }

    }

    public class Tree
    {
        Node rootNode;
        public float lWidth, yL;
        private Node finalNode;
        private int count = 0;
        private bool complete = false;
        public bool enableDrawing = true;

        public Tree(Vector3 pos)
        {
            rootNode = new Node(pos);
        }
        public void ViewParent(Node x)  //  그냥 디버깅용
        {
            Debug.Log("Parent is " + x.parent.position);
        }
        public bool IsComplete()
        {
            return complete;
        }
        public void AddFinalNode(ref Node k) //path를 찾았을 경우
        {
            finalNode = k;
            complete = true;
        }
        public Node FindClosestInChildren(Node x, Vector3 target){
            Node closest = x, temp;
            float closestDistance = Vector3.Distance(x.position, target);
            float checkDistance = 0f;
            if(x.children.Count != 0)
            {
                foreach(Node child in x.children)
                {
                    temp = FindClosestInChildren(child, target);
                    checkDistance = Vector3.Distance(temp.position, target);
                    if(checkDistance < closestDistance)
                    {
                        closestDistance = checkDistance;
                        closest = temp;
                    }
                }
            }
            return closest;
        }
        public Node GetClosestLeaf(Vector3 pos)
        {
            return FindClosestInChildren(rootNode, pos);
        }
        public void DrawLine(Vector3 _start, Vector3 _end, Color color)
        {
            Vector3 start = new Vector3(_start.x, yL, _start.z);
            Vector3 end = new Vector3(_end.x, yL, _end.z);

            GameObject myLine = new GameObject();
            myLine.tag = "path";
            myLine.transform.position = start;
            myLine.AddComponent<LineRenderer>();
            LineRenderer lr = myLine.GetComponent<LineRenderer>();
            lr.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
            lr.startColor = color;
            lr.endColor = color;
            lr.startWidth = lWidth;
            lr.endWidth = lWidth;
            lr.SetPosition(0, start);
            lr.SetPosition(1, end);
        }
        public void DrawCompletedPath(Node x)
        {
            Debug.Log("Drawing Completed Recursive");
            if (!complete)
            {
                Debug.Log("Drawing Completed Recursive");
                return;
            }
            if(x.parent == null)
            {
                Debug.Log("Parent is Null");
                return;
            }
            count++;
            DrawLine(x.parent.position, x.position, Color.red);
            DrawCompletedPath(x.parent);
        }
        public void DrawCompletedPath()
        {
            Debug.Log("Drawing Completed Non Recursive");
            yL = yL + 0.5f;//선 안곂치게 하기
            DrawCompletedPath(finalNode);
            yL = yL - 0.5f;
            Debug.Log("LINES DRAWN:" + count.ToString());
        }

        public Node AddLeaf(ref Node parentLeaf, ref Node childLeaf)
        {
            parentLeaf.AddChild(ref childLeaf);
            if (enableDrawing)
            {
                DrawLine(childLeaf.position, parentLeaf.position, Color.white);
            }
            return childLeaf;
        }

        public void CreateAndAdd(Vector3 position)
        {
            Node nd = new Node(position);
            Node k = GetClosestLeaf(nd.position);
            AddLeaf(ref k, ref nd);
        }
    }

    int IsColliding(Vector3 from, Vector3 to)
    {
        int temp = 0;
        int layerMask = 1;

        RaycastHit hit; 
        Physics.Raycast(from, to -from, out hit ,stepSize, layerMask);

        if(hit.collider != null)
        {
            if(hit.collider.gameObject.tag == "target")
            {
                temp = 1;
            }
            else
            {
                temp = -1;
            }
        }
        return temp;
    }

    Vector3 GetRandomPoint()
    {
        return new Vector3(Random.Range(xLow, xHigh), yLevel ,Random.Range(zLow, zHigh));
    }
}
