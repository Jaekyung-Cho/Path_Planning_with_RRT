using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Dubins : MonoBehaviour
{

    public bool startflag = false;
    [HideInInspector] public Vector3 startPos, goalPos;
    [HideInInspector] public float startHeading, goalHeading, minTuriningRadius = 2.8f / Mathf.Tan(30 * Mathf.Deg2Rad);
    public static float lWidth = 0.2f;
    private float valuechange = 0;

    // Use this for initialization
    void Start()
    {

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (!startflag && Input.GetKey(KeyCode.D))
        {
            startflag = true;
            //Carcontroller carcontroller = GameObject.Find("Car").GetComponent<Carcontroller>();
            //minTuriningRadius = carcontroller.minTurningRadius;
        }
        if (startflag == true)
        {
            Transform starting = GameObject.Find("Car").GetComponent<Transform>();
            Transform Goal = GameObject.FindGameObjectWithTag("target").GetComponent<Transform>();
            startPos = starting.position;
            goalPos = Goal.position;
            startHeading = -starting.rotation.eulerAngles.y * Mathf.Deg2Rad;
            goalHeading = -Goal.rotation.eulerAngles.y * Mathf.Deg2Rad;
            SingleDubinsPath shortest = Dubinspath.GetShortestPath(startPos, startHeading, goalPos, goalHeading, minTuriningRadius);
            if (shortest.totalLength != valuechange)
            {
                GameObject[] deletion = GameObject.FindGameObjectsWithTag("Dubins_once");
                for(int i = 0; i< deletion.Length ; i++){
                    Destroy(deletion[i]);
                }
                shortest.DrawSingleDubinsPath(minTuriningRadius);
				valuechange = shortest.totalLength;
            }
            /*SingleDubinsPath all = Dubinspath.GetAllPaths(startPos, startHeading, goalPos, goalHeading, minTuriningRadius)[0];
            all.DrawSingleDubinsPath(minTuriningRadius);*/
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
		public static List<SingleDubinsPath> GetAllPaths(Vector3 startPos, float startHeading, Vector3 goalPos, float goalHeading, float turnRadius){
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
                allPaths.Add(GetRSR());
            }
            if (startLeftCircle != goalLeftCircle)
            {
                allPaths.Add(GetLSL());
            }
            if ((startRightCircle - goalLeftCircle).magnitude > startCar.turnRadius + goalCar.turnRadius)
            {
                allPaths.Add(GetRSL());
            }
            if ((startLeftCircle - goalRightCircle).magnitude > startCar.turnRadius + goalCar.turnRadius)
            {
                allPaths.Add(GetLSR());
            }
            if ((startRightCircle - goalRightCircle).magnitude < 4 * startCar.turnRadius)
            {
                allPaths.Add(GetRLR());
            }
            if ((startLeftCircle - goalLeftCircle).magnitude < 4 * startCar.turnRadius)
            {
                allPaths.Add(GetLRL());
            }
        }

        private static SingleDubinsPath GetRSR()
        {
            Vector3 startTangent = Vector3.zero;
            Vector3 goalTangent = Vector3.zero;
            float turnRadius = startCar.turnRadius;

            Dubinsmath.RSRorLSL(startRightCircle, goalRightCircle, out startTangent, out goalTangent, turnRadius, false);

            float length1 = Dubinsmath.GetArcLength(startRightCircle, startCar.pos, startTangent, false, turnRadius);
            float length2 = (startTangent - goalTangent).magnitude;
            float length3 = Dubinsmath.GetArcLength(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius);

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
            float length2 = (startTangent - goalTangent).magnitude;
            float length3 = Dubinsmath.GetArcLength(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius);

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
            float length2 = (startTangent - goalTangent).magnitude;
            float length3 = Dubinsmath.GetArcLength(goalLeftCircle, goalTangent, goalCar.pos, true, turnRadius);

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
            float length2 = (startTangent - goalTangent).magnitude;
            float length3 = Dubinsmath.GetArcLength(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius);

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
            float length2 = Dubinsmath.GetArcLength(middleCircle, startTangent, goalTangent, true, turnRadius);
            float length3 = Dubinsmath.GetArcLength(goalRightCircle, goalTangent, goalCar.pos, false, turnRadius);

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

        public void DrawSingleDubinsPath(float turnRadius)
        {
            if (this.pathType == Dubinspath.PathType.RSR)
            {
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, false, Color.green, lWidth);
                Line.DrawLine(tanStart, tanGoal, Color.green, lWidth);
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, false, Color.green, lWidth);
            }
            if (this.pathType == Dubinspath.PathType.LSL)
            {
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, true, Color.green, lWidth);
                Line.DrawLine(tanStart, tanGoal, Color.green, lWidth);
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, true, Color.green, lWidth);
            }
            if (this.pathType == Dubinspath.PathType.RSL)
            {
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, false, Color.green, lWidth);
                Line.DrawLine(tanStart, tanGoal, Color.green, lWidth);
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, true, Color.green, lWidth);
            }
            if (this.pathType == Dubinspath.PathType.LSR)
            {
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, true, Color.green, lWidth);
                Line.DrawLine(tanStart, tanGoal, Color.green, lWidth);
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, false, Color.green, lWidth);
            }
            if (this.pathType == Dubinspath.PathType.RLR)
            {
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, false, Color.green, lWidth);
                Vector3 middleCircle = 2 * tanStart - startCircle;
                Circle.DrawCircle(middleCircle, turnRadius, tanStart, tanGoal, true, Color.green, lWidth);
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, false, Color.green, lWidth);
            }
            if (this.pathType == Dubinspath.PathType.LRL)
            {
                Circle.DrawCircle(startCircle, turnRadius, startPos, tanStart, true, Color.green, lWidth);
                Vector3 middleCircle = 2 * tanStart - startCircle;
                Circle.DrawCircle(middleCircle, turnRadius, tanStart, tanGoal, false, Color.green, lWidth);
                Circle.DrawCircle(goalCircle, turnRadius, tanGoal, goalPos, true, Color.green, lWidth);
            }


            Debug.Log("Pathtype : "+this.pathType+"  total Cost : " + totalLength);
        }
    }

    public static class Line
    {
        public static void DrawLine(Vector3 start, Vector3 end, Color color, float lWidth)
        {
            GameObject myline = new GameObject();
            myline.AddComponent<LineRenderer>();
            LineRenderer line = myline.GetComponent<LineRenderer>();
			myline.tag = "Dubins_once";
            line.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
            line.startColor = color;
            line.endColor = color;
            line.startWidth = lWidth;
            line.endWidth = lWidth;
            line.SetPosition(0, start);
            line.SetPosition(1, end);
        }
    }

    public static class Circle
    {
        public static Vector3 center;
        public static float radius;
        public static Vector3 startPos, endPos;
        public static bool isleft;
        public static int segmentNum = 30;  //modifying if circle is not smooth
        public static void DrawCircle(Vector3 center, float radius, Vector3 startPos, Vector3 endPos, bool isleft, Color color, float lWidth)
        {
            GameObject myline = new GameObject();
            myline.AddComponent<LineRenderer>();
			myline.tag = "Dubins_once";
            LineRenderer line = myline.GetComponent<LineRenderer>();
            line.positionCount = segmentNum + 1;
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
            float angle = Mathf.Atan2(V1.z, V1.x);
            for (int i = 0; i < segmentNum + 1; i++)
            {
                line.SetPosition(i, new Vector3(center.x + radius * Mathf.Cos(angle), center.y, center.z + radius * Mathf.Sin(angle)));
                angle += theta / (segmentNum);
            }
            line.material = new Material(Shader.Find("Particles/Alpha Blended Premultiply"));
            line.startColor = color;
            line.endColor = color;
            line.startWidth = lWidth;
            line.endWidth = lWidth;
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
