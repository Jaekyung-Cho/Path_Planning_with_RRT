using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class makeReferencePath : MonoBehaviour {

	[HideInInspector] public List<Vector3> referencePath = new List<Vector3>();
	[HideInInspector] public List<bool> IsReverse = new List<bool>();
	[HideInInspector] public float GoalCost = 0;
	// Use this for initialization
	void Start () {
		referencePath.Clear();
		IsReverse.Clear();
	}
	
	// Update is called once per frame
	void FixedUpdate () {
		RRT_star_IncludingReverse dubins = GameObject.Find("Car").GetComponent<RRT_star_IncludingReverse>();
		if(dubins.ReferencePath.Count != 0 && GoalCost != dubins.minGoalCost){
			referencePath = dubins.ReferencePath;
			IsReverse = dubins.ReferencePathIsReverse;
			referencePath.Reverse();
			IsReverse.Reverse();
			GoalCost= dubins.minGoalCost;
			Debug.Log("Path updated" + dubins.ReferencePath.Count);
		}
	}
}
