package ev3Localization;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	public Navigation navigation;// initialize navigation object
	private float Rot_Speed = 60; // Rotation speed of the robot
	private double sensorDistFromCenterOfRobot = 13.5;//Distance from the center of rotation to the sensor
	
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		navigation = new Navigation(this.odo);
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
		
		// Hold current X and Y odometer positions
		double xi = this.odo.getX();
		double yi = this.odo.getY();
		
		// Continue loop until a black line is found and that will break the loop
		navigation.setSpeeds(100,100); // go forward at speed 100

		while(true)
		{
			this.colorSensor.fetchSample(colorData, 0); 
			intensity = colorData[0]; // fetch color data
			
			if (intensity  < 0.60){ // if a black line is detected break the while loop
				break;		
			}
		}

		// Get current X and Y odometer positions
		double xf = this.odo.getX();
		
		// Calculate difference between xf and xf in order to obtain the x distance
		// to x=0
		double xDist = xf-xi;
		
		// Stop the robot's motion
		navigation.setSpeeds(0,0);

		// Travel back to the robot's original position
		navigation.setSpeeds(-100,-100);
		while(xf-this.odo.getX() < xDist){}
	
		navigation.setSpeeds(0,0);

		// Rotate 90 degrees counterclockwise in order to be parallel to the y-axis
		navigation.turnTo(90, false);

		// Get current X and Y position
		xi = this.odo.getX();
		yi = this.odo.getY();
		
		// Continue loop until a black line is found and that will break the loop
		navigation.setSpeeds(100,100); // go forward at speed 100
		while(true)
		{
			colorSensor.fetchSample(colorData, 0); 
			intensity = colorData[0]; // fetch color data
			
			if (intensity  < 0.60){ // if a black line is detected break the while loop
				break;		
			}
		}
		
		// Get current Y position
		double yf = this.odo.getY();
		
		// Calculate the distance the robot has traveled in the y direction
		double yDist = yf-yi;
		
		// Stop the robot's motion
		navigation.setSpeeds(0, 0);
		
		// Travel back to the initial X and Y positions
		navigation.setSpeeds(-100,-100);
		while(yf-this.odo.getY() < yDist){}
			
		// Stop the robot's motion
		navigation.setSpeeds(0, 0);
		
		// Calculate the coordinates of the distance the robot needs to travel to
		// based on xDist and yDist, while taking into consideration the position
		// of the light sensor on the robot
		
		navigation.travelTo(xDist-sensorDistFromCenterOfRobot, yDist-sensorDistFromCenterOfRobot);
		navigation.turnTo(0, true);
	}
	

}
