/*
 * Odometer.java
 */

package lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;
	private int leftMotorTachoCount, rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;
	
	public static int lastTachoL;			// Tacho L at last sample
	public static int lastTachoR;			// Tacho R at last sample 
	
	// Constants
	public static final double WHEEL_RADIUS = 2.095;
	public static final double WHEELS_BASE = 16.35;
	
	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		
		
		while (true) {
			updateStart = System.currentTimeMillis();
			//TODO put (some of) your odometer code here
			double distL, distR, deltaD, deltaT, dX, dY;
			
			leftMotorTachoCount = leftMotor.getTachoCount();      					// get tacho counts
			rightMotorTachoCount = rightMotor.getTachoCount();
			distL = 3.14159*WHEEL_RADIUS*(leftMotorTachoCount-lastTachoL)/180;		// compute L and R wheel displacements
			distR = 3.14159*WHEEL_RADIUS*(rightMotorTachoCount-lastTachoR)/180;
			lastTachoL=leftMotorTachoCount;											// save tacho counts for next iteration
			lastTachoR=rightMotorTachoCount;
			deltaD = 0.5*(distL+distR);												// compute vehicle displacement
			deltaT = (distL-distR)/WHEELS_BASE;										// compute change in heading
			
			
			
			synchronized (lock) {
				/**
				 * Don't use the variables x, y, or theta anywhere but here!
				 * Only update the values of x, y, and theta in this block. 
				 * Do not perform complex math
				 * 
				 */
				
				if(deltaT < 0 && theta <= 0 )
				{
					theta=6.27;
				}
				
				theta += deltaT;									// update heading
			    dX = deltaD * Math.sin(theta);						// compute X component of displacement
				dY = deltaD * Math.cos(theta);						// compute Y component of displacement
				x = x + dX;											// update estimates of X and Y position
				y = y + dY;	
				theta=theta%6.27;

			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}
	
	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;	
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;	
		}
	}
}