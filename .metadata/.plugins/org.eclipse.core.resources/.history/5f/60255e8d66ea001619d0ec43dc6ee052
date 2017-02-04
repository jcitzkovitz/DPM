package ev3Loc;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	public Navigation nav;// initialize navigation object
	private float Rot_Speed = 60; // Rotation speed of the robot
	private double distSensor = 15.5;//Distance from the center of rotation to the sensor
	
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		nav = new Navigation(this.odo);
	}
	
	public void doLocalization() {
		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
		
		double angle [] = new double [4]; // array to hold the angles when a gridline is past
		double x,y;
		float intensity; // light intensity variable
		colorSensor.fetchSample(colorData, 0);

		while(Button.waitForAnyPress() != Button.ID_ESCAPE){} // robot waits for user to press button to begin routine
		boolean cont = false;
		nav.turnTo(45,true); // turn 45 degrees
		while(true){ // continue loop untill a black line is found and that will break the loop
			
			nav.setSpeeds(100,100); // go forward at speed 100
			
			colorSensor.fetchSample(colorData, 0); 
			intensity = colorData[0]; // fetch color data
			
			if (intensity  < 0.60){ // if a black line is detected break the while loop
				break;		
			}
		}
		
		nav.setSpeeds(0,0); // stop the robot 
		// putting motors to 0 then to -100 makes the transition between speeds smoother
		nav.setSpeeds(-100,-100); // move backwards 
		Delay.msDelay(4000); // let the robot travel backwards for a specified amount of time found experimentally 
		
		int gridLines = 0; // will be incremented every time the robot detects a black line
		while (gridLines < 4 ){ // same thing done earlier to detect a black line
			colorSensor.fetchSample(colorData, 0);
			nav.setSpeeds(Rot_Speed, -Rot_Speed);
			intensity = colorData[0];
			
			if (intensity > 0.60){
				cont = true;
			}
			
			if (intensity  < 0.60 && cont){
				angle [gridLines] = odo.getAng(); // once a black line is detected save the angle to the angle array
				gridLines++;
				if (gridLines < 4){
					Delay.msDelay(750);
				}
				cont = false;
			}
		}
		nav.setSpeeds(0, 0); // stop the robot
		//The rest is calculations as shown in the lecture slides
		x = Math.abs(angle[2] - angle[0]);
	    y = Math.abs(angle[3] - angle[1]);
		double xpos = -distSensor * Math.cos(Math.toRadians(y) / 2);
		double ypos = -distSensor * Math.cos(Math.toRadians(x) / 2);
		double dtheta = -(angle[3]-180)-((y)/2);
		odo.setPosition(new double [] {xpos, ypos, odo.getAng() + dtheta}, new boolean [] {true, true, true}); // update odometer
		nav.travelTo(0, 0); // travel to (0,0)
		nav.turnTo(0, true); // turn to 0.
	}

}
