using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class button_color : MonoBehaviour {

	[HideInInspector] public float acceleration = 0, angle =0, tempangle = 0;
	[HideInInspector] public bool ReverseGear = false;
	RectTransform steeringWheel;
	Scrollbar forceControl;
	Button D, R, P,speedText;
	
	float maxAcel , maxDecel, speed;
	// Use this for initialization
	void Start () {
		steeringWheel = GameObject.Find("SteeringWheel").GetComponent<RectTransform>();
		forceControl = GameObject.Find("forceControl").GetComponent<Scrollbar>();
		D = GameObject.Find("D").GetComponent<Button>();
		P = GameObject.Find("P").GetComponent<Button>();
		R = GameObject.Find("R").GetComponent<Button>();
		speedText = GameObject.Find("speedText").GetComponent<Button>();
		Carcontroller car = GameObject.Find("Car").GetComponent<Carcontroller>();
		maxAcel = car.accelerationLimit;
		maxDecel = -car.deccelerationLimit;
	}
	
	// Update is called once per frame
	void FixedUpdate () {
		Carcontroller car = GameObject.Find("Car").GetComponent<Carcontroller>();
		acceleration = car.Acceleration;
		ReverseGear = car.ReverseGear;
		speed = car.speed * 3.6f;
		PID pid = GameObject.Find("Car").GetComponent<PID>();
		angle = pid.angle;

		steeringWheel.rotation = Quaternion.Euler(0,0,angle);	
		tempangle = angle;

		if(acceleration >= 0){
			forceControl.value = 0.5f + 0.5f * acceleration / maxAcel;
		}
		else{
			forceControl.value = 0.5f + 0.5f * acceleration / maxDecel;
		}

		if(!ReverseGear){
			D.image.color = Color.red;
			R.image.color = Color.white;
			P.image.color = Color.white;
		}
		else{
			D.image.color = Color.white;
			R.image.color = Color.red;
			P.image.color = Color.white;
		}
		if(!pid.startflag){
			D.image.color = Color.white;
			R.image.color = Color.white;
			P.image.color = Color.red;
		}
		speedText.GetComponentInChildren<Text>().text = Mathf.Round(10*speed)/10+"km/h";

		
	}
}
