package lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator {

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;

	public static void drive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double width, boolean navigate) {
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(1000);
		}

		// wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected that
			// the odometer will be interrupted by another thread
		}

		if(navigate == true)
		{
			
		}
		else
		{
			// Turn -67.08 degrees
			turnTo(63.43,leftMotor, rightMotor, leftRadius, rightRadius, width,true);
			// Go from (0,0) to (60,30)
			travelTo(67.08, leftMotor, rightMotor,leftRadius, rightRadius);
			// Turn 154.43 degrees
			turnTo(156.43,leftMotor, rightMotor, leftRadius, rightRadius, width,false);
			// Go from (60,30) to (30,30)
			travelTo(30, leftMotor, rightMotor,leftRadius, rightRadius);
			// Turn -90 degrees
			turnTo(90,leftMotor, rightMotor, leftRadius, rightRadius, width,true);
			// Go from (30,30) to (30,60)
			travelTo(30, leftMotor, rightMotor,leftRadius, rightRadius);
			// Turn -153.43 degrees
			turnTo(153.43,leftMotor, rightMotor, leftRadius, rightRadius, width,true);
			// Go from (30,60) to (60,0)
			travelTo(67.08, leftMotor, rightMotor,leftRadius, rightRadius);
		}
	}
	
	public static void travelTo(double distance, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius)
	{
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		leftMotor.rotate(convertDistance(leftRadius, distance), true);
		rightMotor.rotate(convertDistance(rightRadius, distance), false);
		
	}
	
	public static void turnTo(double theta, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double leftRadius, double rightRadius, double width, boolean direction)
	{
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		// Turn right
		if(direction == true)
		{
			leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
			rightMotor.rotate(-convertAngle(rightRadius, width, theta), false);
		}
		
		// Turn left
		else
		{
			leftMotor.rotate(-convertAngle(leftRadius, width, theta), true);
			rightMotor.rotate(convertAngle(rightRadius, width, theta), false);
		}
	}
	
	public static boolean isNavigating()
	{
		return true;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
}
