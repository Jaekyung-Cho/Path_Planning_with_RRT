using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RRT_star : MonoBehaviour {
    public float stepSize;
    public float xLow, xHigh, zLow, zHigh;
    public float yLevel, lineWidth;
    public float maxIterations, timestep;
    private Tree theTree;
    public Stack<Node> pathWayPoints = new Stack<Node>();
    private int counter = 0;
    private float timer = 0f;
    public bool enableDrawing = true;
    public bool step = false, disableStepping = false;
    private bool startkey = false;
    private int dimension = 2; // 3차원 드론을 날리면 3으로 바꾸자.(그럴일 없겠지만)
    public float gammaRRT = 140f; //최소값은 2*(1+1/dimension)^(1/dimension) *(area of free space / pi)^(1/dimension) = 138.197
    private Stack<Node> nearX = new Stack<Node>();
    private Stack<Node> nearX_dup = new Stack<Node>();
    public float delta = 5f; // collision 방지를 위한 거리
    private float minCost = 1000000f;
    [HideInInspector] public float minGoalCost = 1000000f;
    private bool startflag = true;
    

    // Use this for initialization
    void Start()
    {
        GameObject[] bc = GameObject.FindGameObjectsWithTag("obstacle");
        for (int i = 0; i < bc.Length; i++)
        {
            BoxCollider box = bc[i].GetComponent<BoxCollider>();
            //Debug.Log(box.name);
            Transform trans = bc[i].GetComponent<Transform>();
            //Debug.Log(trans.lossyScale);
            Vector3 multiplier = new Vector3(1+2*delta/trans.lossyScale.x, 1, 1+2* delta/trans.lossyScale.z);
            //Debug.Log(multiplier);
            box.size = multiplier;
            //box.size.Set(multiplier.x, multiplier.y, multiplier.z);
        }
    }

    // Update is called once per frame
    void FixedUpdate() // 이건 일정한 시간마다 호출되도록 -> 물리엔진에 적합하다
    {
        
        if (Input.GetKey(KeyCode.S) && startflag)
        {
            startkey = true;
            startflag = !startflag;
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
                    float radius = NearRadius(counter, gammaRRT, stepSize, dimension);
                    //Debug.Log(radius);
                    theTree.GetNearLeaf(newpoint.position, radius, ref nearX);
                    Node minX = nearest;
                    minCost = nearest.cost + CostDistance(nearest.position, newpoint.position);
                    newpoint.SetCost(minCost);
                    while (nearX.Count != 0)
                    {
                        Node temp = nearX.Pop();
                        nearX_dup.Push(temp);
                        if (IsColliding(temp.position, newpoint.position) != -1 && temp.cost + CostDistance(temp.position, newpoint.position) < minCost)
                        {
                            minX = temp;
                            minCost = temp.cost + CostDistance(temp.position, newpoint.position);
                            newpoint.SetCost(minCost);
                        }
                    }
                    while(nearX_dup.Count != 0) {
                        Node temp = nearX_dup.Pop();
                        if(IsColliding(newpoint.position, temp.position) != -1 && newpoint.cost + CostDistance(newpoint.position, temp.position) < temp.cost)
                        {
                            temp.parent.children.Remove(temp);
                            theTree.AddLeaf(ref newpoint, ref temp);
                            float costChange = -temp.cost+ newpoint.cost + CostDistance(newpoint.position, temp.position);
                            temp.SetCost(newpoint.cost + CostDistance(newpoint.position, temp.position));
                            costchange(temp, costChange);
                        }
                    }

                    theTree.AddLeaf(ref minX, ref newpoint);

                    if (IsColliding(minX.position, newpoint.position) == 1)
                    { 
                        if(minGoalCost > newpoint.cost){
                            GameObject[] deletion = GameObject.FindGameObjectsWithTag("path");
                            pathWayPoints.Clear();
                            for(int i = 0; i< deletion.Length ; i++){
                                Destroy(deletion[i]);
                            }
                            minGoalCost = newpoint.cost;
                            theTree.AddFinalNode(ref newpoint);
                            theTree.DrawCompletedPath(ref pathWayPoints);
                            Debug.Log("Total Cost : "+minGoalCost + " Iteration : "+counter);
                        }
                    }
                    if(theTree.viewfinalNode()!=null){
                        if(theTree.viewfinalNode().cost < minGoalCost){
                            GameObject[] deletion = GameObject.FindGameObjectsWithTag("path");
                            pathWayPoints.Clear();
                            for(int i = 0; i< deletion.Length ; i++){
                                Destroy(deletion[i]);
                            }
                            minGoalCost = theTree.viewfinalNode().cost;
                            theTree.DrawCompletedPath(ref pathWayPoints);
                            Debug.Log("Total Cost : "+minGoalCost + " Iteration : "+counter);
                        }
                    }
                }
                counter++;
                timer = 0;
            }
            
        }
    }


    public class Node
    {
        public Vector3 position;
        public Node parent;
        public List<Node> children;
        //public GameObject edgeline;
        public float lWidth = 0.1f;
        public float yL;
        public float cost;

        public Node()
        {
            parent = null;
            cost = 0;
            children = new List<Node>();
            position = new Vector3(0, yL, 0);
            //edgeline = new GameObject();
            //edgeline.tag = "edge";
        }
        public Node(Vector3 x)
        {
            parent = null;
            cost = 0;
            children = new List<Node>();
            position = x;
            //edgeline = new GameObject();
            //edgeline.tag = "edge";
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
        public void SetCost(float cost)
        {
            this.cost = cost;
        }
        /*public void DrawLine(Color color)
        {
            if (parent == null)
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
        }*/

    }

    public class Tree
    {
        Node rootNode;
        public float lWidth, yL;
        public Node finalNode;
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
            //complete = true;
        }
        public Node FindClosestInChildren(Node x, Vector3 target)
        {
            Node closest = x, temp;
            float closestDistance = Vector3.Distance(x.position, target);
            float checkDistance = 0f;
            if (x.children.Count != 0)
            {
                foreach (Node child in x.children)
                {
                    temp = FindClosestInChildren(child, target);
                    checkDistance = Vector3.Distance(temp.position, target);
                    if (checkDistance < closestDistance)
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
        public Stack<Node> FindNearInChildren(Node x, Vector3 target, float radius, ref Stack<Node> stack)
        {
            Node temp = x;
            float checkDistance = 0f;
            if(temp.children.Count != 0)
            {
                foreach(Node child in temp.children)
                {
                    FindNearInChildren(child, target, radius, ref stack);
                    checkDistance = Vector3.Distance(child.position, target);
                    if(checkDistance < radius)
                    {
                        stack.Push(child);
                    }
                }
            }
            return stack;
        }
        public Stack<Node> GetNearLeaf(Vector3 pos, float radius, ref Stack<Node> stack)
        {
            float checkDistance = 0f;
            checkDistance = Vector3.Distance(pos, rootNode.position);
            if(checkDistance < radius)
            {
                stack.Push(rootNode);
            }
            return FindNearInChildren(rootNode, pos, radius, ref stack);
        }
        public void DrawLine(Vector3 _start, Vector3 _end, Color color)
        {
            Vector3 start = new Vector3(_start.x, yL, _start.z);
            Vector3 end = new Vector3(_end.x, yL, _end.z);

            GameObject myLine = new GameObject();
            if(color == Color.red)
            {
                myLine.tag = "path";
            }
            if(color == Color.white)
            {
                myLine.tag = "edge";
            }
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
        public void DrawCompletedPath(Node x, ref Stack<Node> pathWayPoints)
        {
            //Debug.Log("Drawing Completed Recursive");
            /*if (!complete)
            {
                //Debug.Log("Drawing Completed Recursive");
                return;
            }*/
            pathWayPoints.Push(x);
            if (x.parent == null)
            {
                
                //Debug.Log("Parent is Null");
                return;
            }
            count++;
            DrawLine(x.parent.position, x.position, Color.red);
            DrawCompletedPath(x.parent, ref pathWayPoints);
        }
        public void DrawCompletedPath(ref Stack<Node> pathWayPoints)
        {
            //Debug.Log("Drawing Completed Non Recursive");
            yL = yL + 0.5f;//선 안곂치게 하기
            DrawCompletedPath(finalNode, ref pathWayPoints);
            yL = yL - 0.5f;
            //Debug.Log("LINES DRAWN:" + count.ToString());
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

        public Node viewfinalNode()
        {
            return finalNode;
        }
    }

    int IsColliding(Vector3 from, Vector3 to)
    {
        int temp = 0;
        int layerMask = 1;

        RaycastHit hit;
        Physics.Raycast(from, to - from, out hit, Mathf.Max(stepSize, (to-from).magnitude), layerMask);

        if (hit.collider != null)
        {
            if (hit.collider.gameObject.tag == "target")
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
        return new Vector3(Random.Range(xLow, xHigh), yLevel, Random.Range(zLow, zHigh));
    }

    float NearRadius(int counter, float gamma, float stepsize, int dimension)
    {
        return Mathf.Min(stepsize, gamma* Mathf.Pow((Mathf.Log(counter)/counter),1/dimension));
    }
    
    float CostDistance(Vector3 from, Vector3 to)
    {
        Vector3 temp = to - from;
        return temp.magnitude;
    }
    void costchange(Node x, float cost)
    {
        Node temp = x;
        if (temp.children.Count != 0)
        {
            foreach (Node child in temp.children)
            {
                costchange(child, cost);
                child.cost = child.cost + cost;
            }
        }
    }
}