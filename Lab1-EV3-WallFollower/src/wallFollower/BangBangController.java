package wallFollower;
import lab3.UltrasonicController;
import lejos.hardware.motor.*;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;

public class BangBangController implements UltrasonicController{
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh;
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int filterControl;
	private final int FILTER_OUT = 50;		
		
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		//Default Constructor
		this.bandCenter = bandCenter;				// Ideal distance from wall (cm)
		this.bandwidth = bandwidth;					// Allowed error from band center (cm)
		this.motorLow = motorLow;					// Low speed motor value (deg/sec)
		this.motorHigh = motorHigh;					// High speed motor value (deg/sec)
		this.leftMotor = leftMotor;					
		this.rightMotor = rightMotor;
		this.filterControl = 0;						// FilterControl count for bad readings
		this.distance = bandCenter;					// Set initial distance to band center
		leftMotor.setSpeed(motorHigh);				// Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
		
	}
	
	@Override
	public void processUSData(int distance) {
		
		// ****Filter Control****
		
		/*Filter control allows for the system to filter out the high values are misunderstood, and utilize
		 * the values that are in fact important. For example, when the robot sees a gap, it might read
		 * an extremely high value, however will not react until the filter allows it too, where by the
		 * time it passes the gap it would have never reacted. On the other hand, if the robot needs to
		 * make a 180 degree turn around a corner, it will notice that it is in fact reading the wanted
		 * value and will make the desired left turn.*/
		
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		
		int distError=this.bandCenter-this.distance;			// Compute error
		
		if (Math.abs(distError) <= this.bandwidth) {			// Within limits, same speed
			
			leftMotor.setSpeed(this.motorHigh);					// Start moving forward
			rightMotor.setSpeed(this.motorHigh);
			leftMotor.forward();
			rightMotor.forward();
		}
		
		else if (distError > 0) {			// ****Too close to the wall****
					
			/*The following 3/4 if-else statements are split up into small increments of distances
			 * from the wall stating when the robot must increase its speed away from the wall 
			 * (to the right) - the closer to the wall, the faster it will move away*/
			
			//The increment/decrement values and distance ranges were chosen through trial and error
			
			if(distError >= 2 && distError < 3)
			{
				leftMotor.setSpeed(this.motorHigh+30);		
				rightMotor.setSpeed(this.motorHigh-30);
				leftMotor.forward();
				rightMotor.forward();
			}
			else if(distError >= 3 && distError < 4)
			{
				leftMotor.setSpeed(this.motorHigh+40);		
				rightMotor.setSpeed(this.motorHigh-40);
				leftMotor.forward();
				rightMotor.forward();
			}
			else if(distError >=5 && distError < 6)
			{
				leftMotor.setSpeed(this.motorHigh+75);		
				rightMotor.setSpeed(this.motorHigh-100);
				leftMotor.forward();
				rightMotor.forward();
			}
			
			 /*The 4th part of the if-else makes a slower and bigger turn away from 
			 * the wall at a closer distance than the others, however still quite far out. The biggest 
			 * turn occurs far out in order to give the robot more time and space to avoid the blocks 
			 * and gaps.*/
			
			else
			{
				leftMotor.setSpeed(this.motorLow+30);		
				rightMotor.setSpeed(this.motorLow);
				leftMotor.forward();
				rightMotor.backward();
			}
			
				
		}
		
		else if (distError < 0) {				// ****Too far from the wall****

			
			/*When reading values just outside of the bandwidth, make the robot
			 * take a slow right turn away from the wall. This corrects situations when the robot
			 * is too far from a corner and headed straight into a wall, as it reads values within
			 * the bandwidth. It does not hurt performance as it allows the robot to keep a safe 
			 * distance from the wall and readjust its sensor readings.*/
			
			// The increment/decrement values were chosen through trial and error
			
			if(Math.abs(distError) >=2 && Math.abs(distError) < 5)
			{
				leftMotor.setSpeed(this.motorLow);		
				rightMotor.setSpeed(this.motorLow);
				leftMotor.forward();
				rightMotor.backward();
			}
			
			/*The following if-else statements describe the situation for the robot turning toward 
			 * the wall (to the left). In these cases the further away the robot deviates from the 
			 * wall, the bigger the turn it makes inward. */
			
			if(Math.abs(distError) >= 5 && Math.abs(distError) < 10)
			{
				leftMotor.setSpeed(this.motorHigh-10);		
				rightMotor.setSpeed(this.motorHigh+10);
				leftMotor.forward();
				rightMotor.forward();
			}
			else if(distError >= 10 && distError < 15)
			{
				leftMotor.setSpeed(this.motorHigh-20);		
				rightMotor.setSpeed(this.motorHigh+20);
				leftMotor.forward();
				rightMotor.forward();
			}
			else if(distError >=15 && distError < 20)
			{
				leftMotor.setSpeed(this.motorHigh-30);		
				rightMotor.setSpeed(this.motorHigh+30);
				leftMotor.forward();
				rightMotor.forward();
			}
			else if(distError >= 20 && distError < 23)
			{
				leftMotor.setSpeed(this.motorHigh-40);		
				rightMotor.setSpeed(this.motorHigh+40);
				leftMotor.forward();
				rightMotor.forward();
			}
			else
			{
				leftMotor.setSpeed(this.motorLow+50);		
				rightMotor.setSpeed(this.motorHigh+50);
				leftMotor.forward();
				rightMotor.forward();
			}
			
		}
	}
		
				
	
	//Print the distance results to the screen
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
