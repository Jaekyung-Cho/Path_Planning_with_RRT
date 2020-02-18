using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RRT_star_Dubinspath1 : MonoBehaviour
{

    public float stepSize;
    public float xLow, xHigh, zLow, zHigh;
    public float yLevel, lineWidth;
    public float maxIterations, timestep;
    private Tree theTree;
    public List<Node> pathWayPoints = new List<Node>();
    public static List<Vector3> AllPathWaypoints = new List<Vector3>();
    [HideInInspector] public List<Vector3> ReferencePath = new List<Vector3>();
    public static float CurveVel = 8f, StraightVel = 10f;
    public static float howDenseCurve = 5, howDenseLine = 0.5f;
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
    private float minCost = float.MaxValue;
    [HideInInspector] public float minGoalCost = 1000000f;
    [HideInInspector] public float minTurningRadius = 1.5f * 2.8f / Mathf.Sin(30 * Mathf.Deg2Rad);
    [HideInInspector] public Vector3 goalPos;
    [HideInInspector] public float goalHeading;
    [HideInInspector] public bool hideflag = false;
    [HideInInspector] public Node hidenNode, minX;
    public float sigma = 4;
    public static bool collisionChecker = true;

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
            Vector3 multiplier = new Vector3(1 + 2 * delta / trans.lossyScale.x, 1, 1 + 2 * delta / trans.lossyScale.z);
            //Debug.Log(multiplier);
            box.size = multiplier;
            //box.size.Set(multiplier.x, multiplier.y, multiplier.z);
        }
        //Carcontroller carcontroller = GameObject.Find("Car").GetComponent<Carcontroller>();
        //minTuriningRadius = carcontroller.minTurningRadius;
    }

    // Update is called once per frame
    void FixedUpdate() // 이건 일정한 시간마다 호출되도록 -> 물리엔진에 적합하다
    {
        if (Input.GetKeyDown(KeyCode.S) && startkey)
        {
            Debug.Log("Stop");
            startkey = false;
        }
        if (Input.GetKeyDown(KeyCode.R) && !startkey)
        {
            PID pid = GameObject.Find("Car").GetComponent<PID>();
            minTurningRadius = pid.minTurningRadius;
            startkey = true;
            theTree = new Tree(new Vector3(transform.position.x, yLevel, transform.position.z), -transform.rotation.eulerAngles.y * Mathf.Deg2Rad);
            theTree.yL = yLevel;
            theTree.lWidth = lineWidth;
            theTree.enableDrawing = enableDrawing;
            Transform Goal = GameObject.FindGameObjectWithTag("target").GetComponent<Transform>();
            goalPos = new Vector3(Goal.position.x, yLevel, Goal.position.z);

            goalHeading = -Goal.rotation.eulerAngles.y * Mathf.Deg2Rad;
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
                if ((counter * 10) % maxIterations == 0 && counter != 0)
                {
                    Debug.Log("Iteration : " + counter);
                }
                /*Vector3 randPoint = new Vector3(0,0,0);
                if(theTree.viewfinalNode() != null){
                    randPoint = GetClosedPoint(pathWayPoints);
                }
                else{
                    randPoint = GetRandomPoint();
                }*/

                Vector3 randPoint = GetRandomPoint();
                //Debug.Log("Random Point" + randPoint.ToString());
                Node nearest = theTree.GetClosestLeaf(randPoint);
                //Debug.Log("Nearest Point" + nearest.position.ToString());
                if (IsColliding(nearest.position, randPoint) != -1)
                {
                    Vector3 direction = randPoint - nearest.position;
                    Node newpoint = new Node(nearest.position + direction / direction.magnitude * stepSize, 0);
                    if ((nearest.position - randPoint).magnitude <= stepSize)
                    {
                        newpoint.SetPosition(randPoint);
                    }
                    newpoint.SetDirection(CalculateDirection(nearest, ref newpoint));
                    if (IsColliding(nearest.position, randPoint) == 1)
                    {
                        newpoint.SetPosition(goalPos);
                        newpoint.SetDirection(goalHeading);
                        newpoint.goal = true;
                    }
                    float radius = NearRadius(counter, gammaRRT, stepSize, dimension);
                    //Debug.Log(radius);
                    theTree.GetNearLeaf(newpoint, radius, ref nearX, minTurningRadius);
                    minX = nearest;
                    minCost = nearest.cost + CostDistance(nearest, newpoint, minTurningRadius);
                    newpoint.SetCost(minCost);
                    while (nearX.Count != 0)
                    {
                        Node temp = nearX.Pop();
                        nearX_dup.Push(temp);
                        if (IsColliding(temp.position, newpoint.position) == 1)
                        {
                            newpoint.SetPosition(goalPos);
                            newpoint.SetDirection(goalHeading);
                            newpoint.goal = true;
                            newpoint.SetCost(float.MaxValue/1000);
                        }
                        float temp_facedirection = newpoint.facedirection;
                        if (!newpoint.goal)
                        {
                            newpoint.SetDirection(CalculateDirection(temp, ref newpoint));
                        }
                        if (IsColliding(temp.position, newpoint.position) != -1 && temp.cost + CostDistance(temp, newpoint, minTurningRadius) < minCost)
                        {
                            minX = temp;
                            minCost = temp.cost + CostDistance(temp, newpoint, minTurningRadius);
                            newpoint.SetCost(minCost);
                        }
                        else
                        {
                            newpoint.SetDirection(temp_facedirection);
                        }
                    }
                    while (nearX_dup.Count != 0)
                    {
                        Node temp = nearX_dup.Pop();
                        float temp_facedirection = temp.facedirection;
                        if (!temp.goal)
                        {
                            temp.SetDirection(CalculateDirection(newpoint, ref temp));
                        }
                        if (IsColliding(newpoint.position, temp.position) != -1 && newpoint.cost + CostDistance(newpoint, temp, minTurningRadius) < temp.cost)
                        {
                            temp.parent.children.Remove(temp);
                            theTree.AddLeaf(ref newpoint, ref temp);

                            float costChange = -temp.cost + newpoint.cost + CostDistance(newpoint, temp, minTurningRadius);
                            temp.SetCost(newpoint.cost + CostDistance(newpoint, temp, minTurningRadius));
                            hidenNode = null;
                            hideflag = false;
                            costchange(temp, costChange);
                        }
                        else
                        {
                            temp.SetDirection(temp_facedirection);
                        }
                    }
                    if(CostDistance(minX,newpoint,minTurningRadius)<float.MaxValue/100000){
                        theTree.AddLeaf(ref minX, ref newpoint);
                    }
                    if(IsColliding(minX.position, newpoint.position) == 1){
                        newpoint.SetPosition(goalPos);
                        newpoint.SetDirection(goalHeading);
                        newpoint.goal = true;
                        newpoint.SetCost(minX.cost + CostDistance(minX,newpoint,minTurningRadius));
                    }
                    if (newpoint.goal && newpoint.cost < float.MaxValue/100000)
                    {
                        if (minGoalCost > newpoint.cost)
                        {
                            Debug.Log(newpoint.parent.position);
                            GameObject[] deletion = GameObject.FindGameObjectsWithTag("Dubins");
                            pathWayPoints.Clear();
                            for (int i = 0; i < deletion.Length; i++)
                            {
                                Destroy(deletion[i]);
                            }
                            minGoalCost = newpoint.cost;
                            theTree.AddFinalNode(ref newpoint);
                            theTree.DrawCompletedPath(ref pathWayPoints, minTurningRadius);
                            ReferencePath = AllPathWaypoints;
                            Debug.Log("Total Cost : " + minGoalCost + " Iteration : " + counter);
                        }
                    }
                    if (hideflag)
                    {
                        if (hidenNode.cost < minGoalCost && hidenNode.cost < float.MaxValue/100000)
                        {
                            Debug.Log(2);
                            GameObject[] deletion = GameObject.FindGameObjectsWithTag("Dubins");
                            pathWayPoints.Clear();
                            for (int i = 0; i < deletion.Length; i++)
                            {
                                Destroy(deletion[i]);
                            }
                            minGoalCost = hidenNode.cost;
                            theTree.AddFinalNode(ref hidenNode);
                            theTree.DrawCompletedPath(ref pathWayPoints, minTurningRadius);
                            ReferencePath = AllPathWaypoints;
                            Debug.Log("Total Cost : " + minGoalCost + " Iteration : " + counter);
                        }
                    }
                    /*if(theTree.viewfinalNode()!=null){
                        if(theTree.viewfinalNode().cost < minGoalCost){
                            GameObject[] deletion = GameObject.FindGameObjectsWithTag("Dubins");
                            pathWayPoints.Clear();
                            for(int i = 0; i< deletion.Length ; i++){
                                Destroy(deletion[i]);
                            }
                            minGoalCost = theTree.viewfinalNode().cost;
                            theTree.DrawCompletedPath(ref pathWayPoints, minTurningRadius);
                            Debug.Log("Total Cost : "+minGoalCost + " Iteration : "+counter);
                        }
                    }*/

                }
                counter++;
                timer = 0;
            }

        }
    }


    //Need improvement
    public static float CalculateDirection(Node from, ref Node to)
    {
        return (from.facedirection + 1.5f * (Mathf.Atan2(to.position.z - from.position.z, to.position.x - from.position.x) - from.facedirection)) % (2 * Mathf.PI);
    }

    public class Path
    {
        public Vector3 position;
        public float velocity;
        public bool isReverse;

        public Path()
        {
            position = Vector3.zero;
            velocity = 0;
            isReverse = false;
        }

        public Path(Vector3 pos, float vel, bool isreverse)
        {
            position = pos;
            velocity = vel;
            isReverse = isreverse;
        }
    }
    public class Node
    {
        public Vector3 position;
        public Node parent;
        public List<Node> children;
        public float facedirection;
        //public GameObject edgeline;
        public float lWidth = 0.1f;
        public float yL;
        public float cost;
        public bool goal;


        public Node()
        {
            parent = null;
            cost = 0;
            children = new List<Node>();
            position = new Vector3(0, yL, 0);
            facedirection = 0;
            goal = false;
            //edgeline = new GameObject();
            //edgeline.tag = "edge";
        }
        public Node(Vector3 x, float direction)
        {
            parent = null;
            cost = 0;
            children = new List<Node>();
            position = x;
            facedirection = direction;
            goal = false;
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
        public void SetPosition(Vector3 pos)
        {
            this.position = pos;
        }
        public void SetDirection(float direction)
        {
            this.facedirection = direction;
        }
    }

    public class Tree
    {
        Node rootNode;
        public float lWidth, yL;
        public Node finalNode;
        private int count = 0;
        private bool complete = false;
        public bool enableDrawing = true;


        public Tree(Vector3 pos, float facedirection)
        {
            rootNode = new Node(pos, facedirection);
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
        public Stack<Node> FindNearInChildren(Node x, Node target, float radius, ref Stack<Node> stack, float turnRadius)
        {
            Node temp = x;
            float checkDistance = 0f;
            if (temp.children.Count != 0)
            {
                foreach (Node child in temp.children)
                {
                    FindNearInChildren(child, target, radius, ref stack, turnRadius);
                    float temp_facedirection = target.facedirection;
                    if (!target.goal)
                    {
                        target.SetDirection(CalculateDirection(temp, ref target));
                    }
                    checkDistance = CostDistance(child, target, turnRadius);
                    if (checkDistance < radius)
                    {
                        stack.Push(child);
                    }
                    else
                    {
                        target.SetDirection(temp_facedirection);
                    }
                }
            }
            return stack;
        }
        public Stack<Node> GetNearLeaf(Node pos, float radius, ref Stack<Node> stack, float turnRadius)
        {
            float checkDistance = 0f;
            float temp_facedirection = pos.facedirection;
            if (!pos.goal)
            {
                pos.SetDirection(CalculateDirection(rootNode, ref pos));
            }
            checkDistance = CostDistance(rootNode, pos, turnRadius);
            if (checkDistance < radius)
            {
                stack.Push(rootNode);
            }
            else
            {
                pos.SetDirection(temp_facedirection);
            }
            return FindNearInChildren(rootNode, pos, radius, ref stack, turnRadius);
        }
        public void DrawLine(Vector3 _start, Vector3 _end, Color color)
        {
            Vector3 start = new Vector3(_start.x, yL, _start.z);
            Vector3 end = new Vector3(_end.x, yL, _end.z);

            GameObject myLine = new GameObject();
            if (color == Color.red)
            {
                myLine.tag = "path";
            }
            if (color == Color.white)
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
        public void DrawCompletedPath(Node x, ref List<Node> pathWayPoints, float turnRadius)
        {
            //Debug.Log("Drawing Completed Recursive");
            /*if (!complete)
            {
                //Debug.Log("Drawing Completed Recursive");
                return;
            }*/
            pathWayPoints.Add(x);
            if (x.parent == null)
            {

                //Debug.Log("Parent is Null");
                return;
            }
            count++;
            SingleDubinsPath draw = Dubinspath.GetShortestPath(x.parent.position, x.parent.facedirection, x.position, x.facedirection, turnRadius);
            if(draw != null){
                draw.DrawSingleDubinsPath(turnRadius, yL);
            }
            DrawCompletedPath(x.parent, ref pathWayPoints, turnRadius);
        }
        public void DrawCompletedPath(ref List<Node> pathWayPoints, float turnRadius)
        {
            //Debug.Log("Drawing Completed Non Recursive");
            yL = yL + 0.5f;//선 안곂치게 하기
            AllPathWaypoints.Clear();
            DrawCompletedPath(finalNode, ref pathWayPoints, turnRadius);
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
            Node nd = new Node(position, 0);
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

        RaycastHit hit1;
        RaycastHit hit2;
        Physics.Raycast(from, to - from, out hit1, Mathf.Max(stepSize, (to - from).magnitude), layerMask);
        Physics.Raycast(to, from - to, out hit2, Mathf.Max(stepSize, (from - to).magnitude), layerMask);

        if (hit1.collider != null)
        {
            if (hit1.collider.gameObject.tag == "target")
            {
                temp = 1;
                if (hit2.collider != null)
                {
                    if (hit2.collider.gameObject.tag != "target")
                    {
                        temp = -1;
                    }

                }
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

    Vector3 GetClosedPoint(List<Node> path)
    {
        Vector3 temp = path[(int)Random.Range(0, (float)(path.Count - 0.01))].position;
        return new Vector3(temp.x + Random.Range(-sigma, sigma), yLevel, temp.z + Random.Range(-sigma, sigma));
    }

    float NearRadius(int counter, float gamma, float stepsize, int dimension)
    {
        return 5 * Mathf.Min(stepsize, gamma * Mathf.Pow((Mathf.Log(counter) / counter), 1 / dimension)); //
    }

    static float CostDistance(Node from, Node to, float turnRadius)
    {
        SingleDubinsPath shortest = Dubinspath.GetShortestPath(from.position, from.facedirection, to.position, to.facedirection, turnRadius);
        if(shortest == null){
            return float.MaxValue/10000;
        }
        return shortest.totalLength;
    }
    void costchange(Node x, float cost)
    {
        Node temp = x;
        if (temp.children.Count != 0)
        {
            foreach (Node child in temp.children)
            {
                costchange(child, cost);
                if (child.goal)
                {
                    hideflag = true;
                    if (hidenNode == null || hidenNode.cost > child.cost)
                    {
                        hidenNode = child;
                    }
                }
                child.SetCost(child.cost + cost);
            }
        }
    }

    public class Car
    {
        public Vector3 pos;
        public float headingInRadians;
        public float turnRadius;
        public Car(Vector3 pos, float headingInRadians, float turnRadius)
        {
            this.pos = pos;
            this.headingInRadians = headingInRadians;
            this.turnRadius = turnRadius;
        }
        public Car(Car car)
        {
            this.pos = car.pos;
            this.headingInRadians = car.headingInRadians;
            this.turnRadius = car.turnRadius;
        }
        public Car changeData(float newPos_x, float newPos_z, float newHeading)
        {
            Car carCopy = new Car(new Vector3(newPos_x, pos.y, newPos_z), newHeading, this.turnRadius);
            return carCopy;
        }
        public float x
        {
            get { return pos.x; }
        }
        public float z
        {
            get { return pos.z; }
        }
        public float heading
        {
            get { return headingInRadians; }
        }
    }
    public static class Dubinspath
    {
        public enum PathType { RSR, LSL, RSL, LSR, RLR, LRL };
        private static Vector3 startLeftCircle;
        private static Vector3 startRightCircle;
        private static Vector3 goalLeftCircle;
        private static Vector3 goalRightCircle;

        private static Car startCar;
        private static Car goalCar;

        public static SingleDubinsPath GetShortestPath(Vector3 startPos, float startHeading, Vector3 goalPos, float goalHeading, float turnRadius)
        {
            startCar = new Car(startPos, startHeading, turnRadius);
            goalCar = new Car(goalPos, goalHeading, turnRadius);

            List<SingleDubinsPath> allPaths = new List<SingleDubinsPath>();
            PositionLeftRightCircle();
            AddAllDubinsPaths(allPaths);

            if (allPaths.Count > 0)
            {
                allPaths.Sort((x, y) => x.totalLength.CompareTo(y.totalLength));
                SingleDubinsPath shortestPath = allPaths[0];

                /*for(int i=0;i<allPaths.Count;i++){
					Debug.Log("pathtype : "+ allPaths[i].pathType+"  cost : "+allPaths[i].totalLength);
				}*/

                //generate 함수가 뭘 의미하는지 모르겠다

                return shortestPath;
            }
            return null;

        }
        public static List<SingleDubinsPath> GetAllPaths(Vector3 startPos, float startHeading, Vector3 goalPos, float goalHeading, float turnRadius)
        {
            startCar = new Car(startPos, startHeading, turnRadius);
            goalCar = new Car(goalPos, goalHeading, turnRadius);

            List<SingleDubinsPath> allPaths = new List<SingleDubinsPath>();
            PositionLeftRightCircle();
            AddAllDubinsPaths(allPaths);

            if (allPaths.Count > 0)
            {
                allPaths.Sort((x, y) => x.totalLength.CompareTo(y.totalLength));


                return allPaths;
            }
            allPaths = null;
            return null;
        }
        static void PositionLeftRightCircle()
        {
            goalRightCircle = Dubinsmath.GetRightCircleCenterPos(goalCar);
            goalLeftCircle = Dubinsmath.GetLeftCircleCenterPos(goalCar);

            startRightCircle = Dubinsmath.GetRightCircleCenterPos(startCar);
            startLeftCircle = Dubinsmath.GetLeftCircleCenterPos(startCar);
        }
        private static void AddAllDubinsPaths(List<SingleDubinsPath> allPaths)
        {
            if (startRightCircle != goalRightCircle)
            {
                if(GetRSR()!=null){
                    allPaths.Add(GetRSR());
                }
            }
            if (startLeftCircle != goalLeftCircle)
            {
                if(GetLSL()!=null){
                    allPaths.Add(GetLSL());
                }
            }
            if ((startRightCircle - goalLeftCircle).magnitude > startCar.turnRadius + goalCar.turnRadius)
            {
                if(GetRSL()!=null){
                    allPaths.Add(GetRSL());
                }
            }
            if ((startLeftCircle - goalRightCircle).magnitude > startCar.turnRadius + goalCar.turnRadius)
            {
                if(GetLSR()!=null){
                    allPaths.Add(GetLSR());
                }  
            }
            if ((startRightCircle - goalRightCircle).magnitude < 4 * startCar.turnRadius)
            {
                if(GetRLR()!=null){
                    allPaths.Add(GetRLR());
                }
            }
            if ((startLeftCircle - goalLeftCircle).magnitude < 4 * startCar.turnRadius)
            {
                if(GetLRL()!=null){
                    allPaths.Add(GetLRL());
                }
            }
        }

        private static SingleDubinsPath GetRSR()
        {
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            float turnRadius = startCar.turnRadius;

            Dubinsmath.RSRorLSL(startRightCircle, goalRightCircle, out startTangent, out goalTangent, turnRadius, false);

            float length1 = Dubinsmath.GetArcLength(startRightCircle, startCar.pos, startTangent, false, turnRadius);
            if(Circle.checkCircle(startRightCircle, startCar.pos, startTangent, false, turnRadius)){
                return null;
            }
            float length2 = (startTangent - goalTangent).magnitude;
            if(Line.checkLine(startTangent, goalTangent)){
                return null;
            }
            float length3 = Dubinsmath.GetArcLength(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius);
            if(Circle.checkCircle(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius)){
                return null;
            }

            SingleDubinsPath pathData = new SingleDubinsPath(length1 + length2 + length3, startRightCircle, startCar.pos,
                                                            goalRightCircle, goalCar.pos, startTangent, goalTangent, PathType.RSR);

            return pathData;
        }

        private static SingleDubinsPath GetLSL()
        {
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            float turnRadius = startCar.turnRadius;

            Dubinsmath.RSRorLSL(startLeftCircle, goalLeftCircle, out startTangent, out goalTangent, turnRadius, true);

            float length1 = Dubinsmath.GetArcLength(startLeftCircle, startCar.pos, startTangent, true, turnRadius);
            if(Circle.checkCircle(startLeftCircle, startCar.pos, startTangent, true, turnRadius)) {
                return null;
            }
            float length2 = (startTangent - goalTangent).magnitude;
            if(Line.checkLine(startTangent, goalTangent)){
                return null;
            }
            float length3 = Dubinsmath.GetArcLength(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius);
            if(Circle.checkCircle(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius)){
                return null;
            }

            SingleDubinsPath pathData = new SingleDubinsPath(length1 + length2 + length3, startLeftCircle, startCar.pos,
                                                            goalLeftCircle, goalCar.pos, startTangent, goalTangent, PathType.LSL);

            return pathData;
        }

        private static SingleDubinsPath GetRSL()
        {
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            float turnRadius = startCar.turnRadius;

            Dubinsmath.RSLorLSR(startRightCircle, goalLeftCircle, out startTangent, out goalTangent, turnRadius, false);

            float length1 = Dubinsmath.GetArcLength(startRightCircle, startCar.pos, startTangent, false, turnRadius);
            if(Circle.checkCircle(startRightCircle, startCar.pos, startTangent, false, turnRadius)){
                return null;
            }
            float length2 = (startTangent - goalTangent).magnitude;
            if(Line.checkLine(startTangent, goalTangent)){
                return null;
            }
            float length3 = Dubinsmath.GetArcLength(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius);
            if(Circle.checkCircle(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius)){
                return null;
            }

            SingleDubinsPath pathData = new SingleDubinsPath(length1 + length2 + length3, startRightCircle, startCar.pos,
                                                             goalLeftCircle, goalCar.pos, startTangent, goalTangent, PathType.RSL);

            return pathData;
        }
        private static SingleDubinsPath GetLSR()
        {
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            float turnRadius = startCar.turnRadius;

            Dubinsmath.RSLorLSR(startLeftCircle, goalRightCircle, out startTangent, out goalTangent, turnRadius, true);

            float length1 = Dubinsmath.GetArcLength(startLeftCircle, startCar.pos, startTangent, true, turnRadius);
            if(Circle.checkCircle(startLeftCircle, startCar.pos, startTangent, true, turnRadius)){
                return null;
            }
            float length2 = (startTangent - goalTangent).magnitude;
            if(Line.checkLine(startTangent,goalTangent)){
                return null;
            }
            float length3 = Dubinsmath.GetArcLength(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius);
            if(Circle.checkCircle(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius)){
                return null;
            }

            SingleDubinsPath pathData = new SingleDubinsPath(length1 + length2 + length3, startLeftCircle, startCar.pos,
                                                             goalRightCircle, goalCar.pos, startTangent, goalTangent, PathType.LSR);

            return pathData;
        }

        private static SingleDubinsPath GetRLR()
        {
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            Vector3 middleCircle = Vector3.zero;
            float turnRadius = startCar.turnRadius;

            Dubinsmath.RLRorLRL(startRightCircle, goalRightCircle, out startTangent, out goalTangent, out middleCircle, turnRadius, true);

            float length1 = Dubinsmath.GetArcLength(startRightCircle, startCar.pos, startTangent, false, turnRadius);
            if(Circle.checkCircle(startRightCircle, startCar.pos, startTangent, false, turnRadius)){
                return null;
            }
            float length2 = Dubinsmath.GetArcLength(middleCircle, startTangent, goalTangent, true, turnRadius);
            if(Circle.checkCircle(middleCircle, startTangent, goalTangent, true, turnRadius)){
                return null;
            }
            float length3 = Dubinsmath.GetArcLength(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius);
            if(Circle.checkCircle(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius)){
                return null;
            }

            SingleDubinsPath pathData = new SingleDubinsPath(length1 + length2 + length3, startRightCircle, startCar.pos,
                                                            goalRightCircle, goalCar.pos, startTangent, goalTangent, PathType.RLR);

            return pathData;
        }
        private static SingleDubinsPath GetLRL()
        {
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            Vector3 middleCircle = Vector3.zero;
            float turnRadius = startCar.turnRadius;

            Dubinsmath.RLRorLRL(startLeftCircle, goalLeftCircle, out startTangent, out goalTangent, out middleCircle, turnRadius, false);

            float length1 = Dubinsmath.GetArcLength(startLeftCircle, startCar.pos, startTangent, true, turnRadius);
            float length2 = Dubinsmath.GetArcLength(middleCircle, startTangent, goalTangent, false, turnRadius);
            float length3 = Dubinsmath.GetArcLength(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius);
            if(Circle.checkCircle(startLeftCircle, startCar.pos, startTangent, true, turnRadius)
             || Circle.checkCircle(middleCircle, startTangent, goalTangent, false, turnRadius) 
            || Circle.checkCircle(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius)){
                return null;
            }

            SingleDubinsPath pathData = new SingleDubinsPath(length1 + length2 + length3, startLeftCircle, startCar.pos,
                                                            goalLeftCircle, goalCar.pos, startTangent, goalTangent, PathType.LRL);

            return pathData;
        }
    }

    public class SingleDubinsPath
    {
        public float totalLength;
        public Vector3 startCircle;
        public Vector3 startPos;
        public Vector3 goalCircle;
        public Vector3 goalPos;
        public Vector3 tanStart;
        public Vector3 tanGoal;
        public Dubinspath.PathType pathType;


        public SingleDubinsPath(float totalLength, Vector3 startCircle, Vector3 startPos, Vector3 goalCircle, Vector3 goalPos, Vector3 tanStart, Vector3 tanGoal, Dubinspath.PathType pathtype)
        {
            this.totalLength = totalLength;

            this.startCircle = startCircle;
            this.startPos = startPos;
            this.goalCircle = goalCircle;
            this.goalPos = goalPos;
            this.tanStart = tanStart;
            this.tanGoal = tanGoal;
            this.pathType = pathtype;
        }

        public void DrawSingleDubinsPath(float turnRadius, float yL)
        {
            float lWidth = 0.2f;
            Debug.Log("pathtype" + this.pathType + "  total length : " + this.totalLength);
            if (this.pathType == Dubinspath.PathType.RSR)
            {
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, false, Color.blue, lWidth, yL);
                Line.DrawLine(tanStart, tanGoal, Color.blue, lWidth, yL);
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, false, Color.blue, lWidth, yL);
            }
            if (this.pathType == Dubinspath.PathType.LSL)
            {
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, true, Color.blue, lWidth, yL);
                Line.DrawLine(tanStart, tanGoal, Color.blue, lWidth, yL);
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, true, Color.blue, lWidth, yL);
            }
            if (this.pathType == Dubinspath.PathType.RSL)
            {
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, true, Color.blue, lWidth, yL);
                Line.DrawLine(tanStart, tanGoal, Color.blue, lWidth, yL);
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, false, Color.blue, lWidth, yL);
            }
            if (this.pathType == Dubinspath.PathType.LSR)
            {
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, false, Color.blue, lWidth, yL);
                Line.DrawLine(tanStart, tanGoal, Color.blue, lWidth, yL);
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, true, Color.blue, lWidth, yL);
            }
            if (this.pathType == Dubinspath.PathType.RLR)
            {
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, false, Color.blue, lWidth, yL);
                Vector3 middleCircle = 2 * tanStart - startCircle;
                Circle.DrawCircle(middleCircle, turnRadius, tanStart, tanGoal, true, Color.blue, lWidth, yL);
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, false, Color.blue, lWidth, yL);
            }
            if (this.pathType == Dubinspath.PathType.LRL)
            {
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, true, Color.blue, lWidth, yL);
                Vector3 middleCircle = 2 * tanStart - startCircle;
                Circle.DrawCircle(middleCircle, turnRadius, tanStart, tanGoal, false, Color.blue, lWidth, yL);
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, true, Color.blue, lWidth, yL);
            }


            //Debug.Log("Pathtype : "+this.pathType+"  total Cost : " + totalLength);
        }
    }

    public static class Line
    {
        public static void DrawLine(Vector3 start, Vector3 end, Color color, float lWidth, float yL)
        {
            GameObject myline = new GameObject();
            myline.AddComponent<LineRenderer>();
            LineRenderer line = myline.GetComponent<LineRenderer>();
            myline.tag = "Dubins";
            line.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
            line.startColor = color;
            line.endColor = color;
            line.startWidth = lWidth;
            line.endWidth = lWidth;
            line.SetPosition(0, new Vector3(start.x, yL, start.z));
            line.SetPosition(1, new Vector3(end.x, yL, end.z));


            int i = 0;
            Vector3 temp = Vector3.zero;
            while (i * howDenseLine < (start - end).magnitude)
            {
                temp = end + (start - end).normalized * i * howDenseLine;
                AllPathWaypoints.Add(new Vector3(temp.x, 0, temp.z));
                i++;
            }
        }
        public static bool checkLine(Vector3 start, Vector3 end){
            if(Iscolliding(start, end)==1){
                return true;
            }
            return false;
        }
        public static int Iscolliding(Vector3 from, Vector3 to)
        {
            int temp = 0;
            int layerMask = 1;

            from.y = 0.5f;
            to.y = 0.5f;

            RaycastHit hit1;
            Physics.Raycast(from, to - from, out hit1, (to - from).magnitude, layerMask);

            if (hit1.collider != null)
            {
                if (hit1.collider.gameObject.tag == "obstacle")
                {
                    temp = 1;
                }
            }
            return temp;
        }
    }

    public static class Circle
    {
        public static Vector3 center;
        public static float radius;
        public static Vector3 startPos, endPos;
        public static bool isleft;
        public static int segmentNum = 30;  //modifying if circle is not smooth
        public static float yL;
        public static void DrawCircle(Vector3 center, float radius, Vector3 startPos, Vector3 endPos, bool isleft, Color color, float lWidth, float yL)
        {
            GameObject myline = new GameObject();
            myline.AddComponent<LineRenderer>();
            myline.tag = "Dubins";
            LineRenderer line = myline.GetComponent<LineRenderer>();
            line.useWorldSpace = false;
            Vector3 V1 = startPos - center;
            Vector3 V2 = endPos - center;
            float theta = Mathf.Atan2(V2.z, V2.x) - Mathf.Atan2(V1.z, V1.x);
            if (isleft && theta < 0)
            {
                theta += 2 * Mathf.PI;
            }
            else if (!isleft && theta > 0)
            {
                theta -= 2 * Mathf.PI;
            }
            //for dense waypoints in curve, change "mathf.PI / 36" 
            segmentNum = (int)(Mathf.Abs(theta) / (howDenseCurve * Mathf.PI / 180)) + 1;

            line.positionCount = segmentNum + 1;
            float angle = Mathf.Atan2(V1.z, V1.x) + theta;
            for (int i = 0; i < segmentNum + 1; i++)
            {
                line.SetPosition(i, new Vector3(center.x + radius * Mathf.Cos(angle), yL, center.z + radius * Mathf.Sin(angle)));
                AllPathWaypoints.Add(new Vector3(center.x + radius * Mathf.Cos(angle), 0, center.z + radius * Mathf.Sin(angle)));
                angle -= theta / (segmentNum);
            }
            line.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
            line.startColor = color;
            line.endColor = color;
            line.startWidth = lWidth;
            line.endWidth = lWidth;
        }

        public static bool checkCircle(Vector3 center, Vector3 startPos, Vector3 endPos, bool isleft, float radius){
            Vector3 V1 = startPos - center;
            Vector3 V2 = endPos - center;
            float theta = Mathf.Atan2(V2.z, V2.x) - Mathf.Atan2(V1.z, V1.x);
            if (isleft && theta < 0)
            {
                theta += 2 * Mathf.PI;
            }
            else if (!isleft && theta > 0)
            {
                theta -= 2 * Mathf.PI;
            }
            //for dense waypoints in curve, change "mathf.PI / 36" 
            segmentNum = (int)(Mathf.Abs(theta) / (howDenseCurve * 10 * Mathf.PI / 180)) + 1;

            float angle = Mathf.Atan2(V1.z, V1.x) + theta;
            Vector3 tempVec = endPos;
            Vector3 newVec = endPos;
            for (int i = 0; i < segmentNum + 1; i++)
            {
                tempVec = new Vector3(center.x + radius * Mathf.Cos(angle), yL, center.z + radius * Mathf.Sin(angle));
                angle -= theta / (segmentNum);
                newVec = new Vector3(center.x + radius * Mathf.Cos(angle), yL, center.z + radius * Mathf.Sin(angle));
                if (Iscolliding(newVec, tempVec) == 1)
                {
                    return true;
                }
            }
            return false;
        }
        public static int Iscolliding(Vector3 from, Vector3 to)
        {
            int temp = 0;
            int layerMask = 1;

            RaycastHit hit1;
            Physics.Raycast(from, to - from, out hit1, (to - from).magnitude, layerMask);

            if (hit1.collider != null)
            {
                if (hit1.collider.gameObject.tag == "obstacle")
                {
                    temp = 1;
                }
            }
            return temp;
        }
    }



    public static class Dubinsmath
    {
        public static Vector3 GetRightCircleCenterPos(Car car)
        {
            Vector3 rightCirclePos = Vector3.zero;

            rightCirclePos.x = car.pos.x + car.turnRadius * Mathf.Cos(car.headingInRadians - Mathf.PI / 2);
            rightCirclePos.y = car.pos.y;
            rightCirclePos.z = car.pos.z + car.turnRadius * Mathf.Sin(car.headingInRadians - Mathf.PI / 2);

            return rightCirclePos;
        }
        public static Vector3 GetLeftCircleCenterPos(Car car)
        {
            Vector3 leftCirclePos = Vector3.zero;

            leftCirclePos.x = car.pos.x + car.turnRadius * Mathf.Cos(car.headingInRadians + Mathf.PI / 2);
            leftCirclePos.y = car.pos.y;
            leftCirclePos.z = car.pos.z + car.turnRadius * Mathf.Sin(car.headingInRadians + Mathf.PI / 2);

            return leftCirclePos;
        }

        //RSR and LSL for tangent node
        //startPos와 goalPos는 원의 중심을 의미한다
        public static void RSRorLSL(Vector3 startPos, Vector3 goalPos, out Vector3 startTangent, out Vector3 goalTangent, float turnRadius, bool LSL)
        {
            float theta = 90 * Mathf.Deg2Rad;
            theta += Mathf.Atan2(goalPos.z - startPos.z, goalPos.x - startPos.x);
            if (LSL)
            {
                theta += Mathf.PI;
            }
            float x_st = startPos.x + turnRadius * Mathf.Cos(theta);
            float z_st = startPos.z + turnRadius * Mathf.Sin(theta);

            Vector3 direction = goalPos - startPos;

            float x_gt = x_st + direction.x;
            float z_gt = z_st + direction.z;

            startTangent = new Vector3(x_st, startPos.y, z_st);
            goalTangent = new Vector3(x_gt, goalPos.y, z_gt);
        }

        public static void RSLorLSR(Vector3 startPos, Vector3 goalPos, out Vector3 startTangent, out Vector3 goalTangent, float turnRadius, bool LSR)
        {
            float D = (goalPos - startPos).magnitude;
            float theta = Mathf.Acos(2 * turnRadius / D);
            if (LSR)
            {
                theta = -theta;
            }
            theta += Mathf.Atan2(goalPos.z - startPos.z, goalPos.x - startPos.x);
            float x_st = startPos.x + turnRadius * Mathf.Cos(theta);
            float z_st = startPos.z + turnRadius * Mathf.Sin(theta);

            Vector3 direction = goalPos - new Vector3(x_st + turnRadius * Mathf.Cos(theta), 0, z_st + turnRadius * Mathf.Sin(theta));

            float x_gt = x_st + direction.x;
            float z_gt = z_st + direction.z;

            startTangent = new Vector3(x_st, startPos.y, z_st);
            goalTangent = new Vector3(x_gt, goalPos.y, z_gt);
        }

        public static void RLRorLRL(Vector3 startPos, Vector3 goalPos, out Vector3 startTangent, out Vector3 goalTangent, out Vector3 middleCircle, float turnRadius, bool RLR)
        {
            float D = (goalPos - startPos).magnitude;
            float theta = Mathf.Acos(D / (4 * turnRadius));
            if (RLR)
            {
                theta = -theta;
            }
            theta += Mathf.Atan2(goalPos.z - startPos.z, goalPos.x - startPos.x);

            middleCircle = new Vector3(startPos.x + 2 * turnRadius * Mathf.Cos(theta), startPos.y, startPos.z + 2 * turnRadius * Mathf.Sin(theta));

            startTangent = (startPos + middleCircle) / 2f;
            goalTangent = (middleCircle + goalPos) / 2f;
        }

        public static float GetArcLength(Vector3 circleCenter, Vector3 startPos, Vector3 goalPos, bool leftcircle, float turnRadius)
        {
            Vector3 V1 = startPos - circleCenter;
            Vector3 V2 = goalPos - circleCenter;

            float theta = Mathf.Atan2(V2.z, V2.x) - Mathf.Atan2(V1.z, V1.x);
            if (leftcircle && theta < 0)
            {
                theta += 2 * Mathf.PI;
            }
            else if (!leftcircle && theta > 0)
            {
                theta -= 2 * Mathf.PI;
            }
            float arcLength = turnRadius * Mathf.Abs(theta);
            return arcLength;
        }
    }
}
