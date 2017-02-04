package ev3Loc;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static float ROTATION_SPEED = 120;
	
	private static final int LEEWAY	= 33; // leeway in the reading of the US 

	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	public Navigation nav; // Initialize navigation object
	
	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationType locType) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		nav = new Navigation(this.odo);
	}
	
	public void doLocalization() {
		double angleA, angleB; // variable for agnle b
		double actualAng = 0; // the actual angle of the robot used at the end to position robot 
		
		if (locType == LocalizationType.FALLING_EDGE) {
			
			// rotate the robot until it sees no wall
			while(getFilteredData() < LEEWAY){
				rightTurn();
			}
			Delay.msDelay(1000);// Delay to avoid getting bad readings from the sensor
			
			//Rotate robot untill a wall is seen
			while (getFilteredData() >= LEEWAY){	
				rightTurn();
			}
			angleA = odo.getAng(); // latch the angle
			nav.setSpeeds(0, 0);//stop the robto
			Delay.msDelay(1000);// avoid getting bad readings from the sensor
			
			// switch direction and wait until it sees no wall
			while(getFilteredData() < LEEWAY){
				leftTurn();
			}
			Delay.msDelay(1000); // delay to avoid getting bad readings
			
			// keep rotating until the robot sees a wall, then latch the angle
			while (getFilteredData() >= LEEWAY){
				leftTurn();
			}
			nav.setSpeeds(0, 0); // stop the robot
			angleB = odo.getAng(); // latch the angle
			Delay.msDelay(1000);
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			
			
			actualAng = calcHeading(angleA,angleB) + odo.getAng(); // get the actual angle of the robot
			// update the odometer position
			odo.setPosition(new double [] {0.0, 0.0, actualAng}, new boolean [] {true, true, true});
			nav.turnTo(0, true); // turn to 0
			
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			// rotate the robot until it sees  wall
			
			while(getFilteredData() > LEEWAY){
				rightTurn();
			}
			Delay.msDelay(1000);
						
			// keep rotating until the robot sees a no wall, then latch the angle
			while (getFilteredData() <= LEEWAY){
				rightTurn();
			}
						
			angleA = odo.getAng();
			nav.setSpeeds(0, 0);
			Delay.msDelay(1000);
			// switch direction and wait until it sees wall
						
			while(getFilteredData() > LEEWAY){
				leftTurn();
			}
			Delay.msDelay(1000);
			
			// keep rotating until the robot sees no wall, then latch the angle
			while (getFilteredData() <= LEEWAY){
				leftTurn();
			}
			nav.setSpeeds(0, 0);
			angleB = odo.getAng();
			Delay.msDelay(1000);
						
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
						
						
			actualAng = calcHeading(angleB,angleA) + odo.getAng();
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, actualAng}, new boolean [] {true, true, true});
			//nav.setSpeeds(ROTATION_SPEED, ROTATION_SPEED);
			nav.turnTo(0, true);
		}
	}
	
	private float getFilteredData() { // filter the data 
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
		
		if (distance >= 50){
			distance = 50;
		}
		return distance;
	}
	
	private  void leftTurn(){ // rotate left
		nav.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);
	}

	private  void rightTurn(){ // rotate right
		nav.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);
	}
	
	private static double calcHeading(double angleA, double angleB){ // calculation of heading as shown in the tutorial
		
		double heading;
		
		if (angleA < angleB){
			heading = 45 - (angleA + angleB) /2 ;
		}else {
			heading = 225 - (angleA + angleB) /2 ;	
		}
		return heading;
	}
}
