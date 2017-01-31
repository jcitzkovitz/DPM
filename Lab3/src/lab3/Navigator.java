package lab3;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigator {

	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	double leftRadius; 
	double rightRadius; 
	double width; 
	boolean navigate;
	Odometer o;
	PController p;
	UltrasonicPoller usPoller;
	Thread curThread;
	volatile double xDestOnCurrentPath;
	volatile double yDestOnCurrentPath;
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	volatile int[][] pointsToTravelTo = new int[2][2];
	
	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double width, boolean navigate, Odometer o, 
			UltrasonicPoller usPoller, Thread curThread, PController p)
			{
				this.leftMotor = leftMotor;
				this.rightMotor = rightMotor;
				this.leftRadius = leftRadius;
				this.rightRadius = rightRadius;
				this.width = width;
				this.navigate = navigate;
				this.o = o;
				this.usPoller = usPoller;
				this.curThread = curThread;
				this.xDestOnCurrentPath = 0;
				this.yDestOnCurrentPath = 0;
				this.p = p;
			}
	
	public void drive() {
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor }) {
			motor.stop();
			motor.setAcceleration(1000);
		}

		// Wait 5 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// There is nothing to be done here because it is not expected that
			// The odometer will be interrupted by another thread
		}

		if(navigate == true)
		{
			
			// Set points to travel to in order to access where is left to travel in a thread that will be 
			// spawned to run a wall avoidance method
			pointsToTravelTo[0][0]=0;
			pointsToTravelTo[0][1]=60;
			pointsToTravelTo[1][0]=60;
			pointsToTravelTo[1][1]=0;
			
			//ObstacleAvoidance obstacleAvoidanceThread = new ObstacleAvoidance();
			//obstacleAvoidanceThread.start();
			
			// This thread will use the ulltrasonic sensor to detect an obstacle, and maneouver around it
			// by using the p controller.
			(new Thread() {
				public void run() {
					boolean Continue = true;
					int dist = 0;
					while(Continue)
					{
						dist = usPoller.getDistance();
						if(dist <= 17)
						{
							// Interupt the current thread when the robot is too close to the obstacle
							// as you dont want the two threads competing over the robot's motion.
							curThread.interrupt();
							
							// Calculate the the robots line of travel using odometer and current 
							// destination values
							double[] function = calculateLineToPathFunction();
							
							// Set the p controller processData boolean to true in order to commence 
							// the p controller's data processing method
							p.setProcessData(true);
							
							sensorMotor.setSpeed(50);
							sensorMotor.rotate(45);
							
							// Sleep for several seconds in order to avoid comparing the current points
							// to the function calculated above.
							try{Thread.sleep(2000);}catch(Exception e){};
							
							boolean onLineToPath = false;
							while(!onLineToPath)
								{
									// Check if the robot is, within error, back on the same path prior
									// to deviation
									onLineToPath = functionIsBackOnLineToPath(function,o.getX(),o.getY());
								}
							
							p.setProcessData(false);
							sensorMotor.rotate(-45,false);
							usPoller.interrupt();
							
							// If the first point has been traversed, only travel to the second point
							if(pointsToTravelTo[0][0]==-1)
							{
								travelTo(pointsToTravelTo[1][0],pointsToTravelTo[1][1]);
							}
							
							// Else, continue traveling to the first point and then the second point
							else
							{
								travelTo(pointsToTravelTo[0][0],pointsToTravelTo[0][1]);
								travelTo(pointsToTravelTo[1][0],pointsToTravelTo[1][1]);
							}
							
							Continue = false;
						}
					}
				}
			}).start();
			
			
			// Set x and y positions of where the robot is planning on travelling to next
			xDestOnCurrentPath = 0;
			yDestOnCurrentPath = 60;
			
			//Drive from (0,0) to (60,30)
			travelTo(0,60);	
			
			// Set a marker for the thread running the wall avoider in order to notify
			// which point it was traveling to prior to diverting
			pointsToTravelTo[0][0]=-1;
					
			// Set x and y positions of where the robot is planning on travelling to next
			xDestOnCurrentPath = 60;
			yDestOnCurrentPath = 0;
			
			//Drive from (60,30) to (30,30)
			travelTo(60,0);
			
		}
		else
		{
			
			//Drive from (0,0) to (60,30)
			travelTo(60,30);
				
			//Drive from (60,30) to (30,30)
			travelTo(30,30);
					
			//Drive from (30,30) to (30,60)
			travelTo(30,60);
				
			//Drive from (30,60) to (60,0)
			travelTo(60,0);
		}
	}
	
	public void travelTo(int xf, int yf)
	{
		int xi = (int) o.getX();
		int yi = (int) o.getY();
		double theta = calculateThetaForTurn(xi,yi,xf,yf);
		turnTo(theta);
		double dist = Math.sqrt(Math.pow(xf-xi,2)+Math.pow(yf-yi,2));
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(leftRadius, dist), true);
		rightMotor.rotate(convertDistance(rightRadius, dist), false);
	}
	
	public void turnTo(double theta)
	{
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		theta=theta*180/Math.PI;
		leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, theta), false);
		
	}
	
	public double calculateThetaForTurn(double xi, double yi, double xf, double yf)
	{
		double diffX = xf-xi;
		double diffY = yf-yi;
		double theta = 0;
		double curTheta = o.getTheta();
		double axisTheta = 0;
		
		// Traveling to positive x and y (Quadrant 1)
		if(diffX > 0 && diffY > 0)
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = Math.PI/2-theta;
			
			double theta1 = 2*Math.PI-curTheta+axisTheta;
			double theta2 = curTheta-axisTheta;
				
			if(Math.abs(theta1)<Math.abs(theta2))
			{
				theta = theta1;
			}
			else
			{
				theta = -theta2;
			}		
		}
		
		// Traveling to positive x and negative y (Quadrant 4)
		else if(diffX > 0 && diffY < 0)
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = Math.PI/2-theta;
			
			double theta1 = 2*Math.PI-curTheta+axisTheta;
			double theta2 = curTheta-axisTheta;
			
			if(Math.abs(theta1)<Math.abs(theta2))
			{
				theta = theta1;
			}
			else
			{
				theta = -theta2;
			}
		}
		
		// Traveling to negative x and negative y (Quadrant 3)
		else if(diffX < 0 && diffY < 0)			
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = 3*Math.PI/2-theta;
			
			double theta1 = 2*Math.PI-curTheta+axisTheta;
			double theta2 = curTheta-axisTheta;
				
			if(Math.abs(theta1)<Math.abs(theta2))
			{
				theta = theta1;
			}
			else
			{
				theta = -theta2;
			}
			
		}
		
		// Traveling to negative x and positive y (Quadrant 2)
		else if(diffX < 0 && diffY > 0)
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = 3*Math.PI/2-theta;
			
			double theta1 = 2*Math.PI-curTheta+axisTheta;
			double theta2 = curTheta-axisTheta;
				
			if(Math.abs(theta1)<Math.abs(theta2))
			{
				theta = theta1;
			}
			else
			{
				theta = -theta2;
			}

		}
		
		// If there is no change in x, only travel in the y direction
		else if(diffX == 0)
		{
			// Travel in the positive y direction
			if(diffY > 0)
			{
				if(curTheta >= 0 && curTheta <= Math.PI)
				{
					theta = -curTheta;
				}
				else if(curTheta > Math.PI && curTheta <= 2*Math.PI)
				{
					theta = 2*Math.PI-curTheta;
				}

			}
			
			// Travel in the negative y direction
			else if (diffY < 0)
			{
				if(curTheta >= 0 && curTheta <= Math.PI)
				{
					theta = Math.PI-curTheta;
				}
				else if(curTheta > Math.PI && curTheta <= 2*Math.PI)
				{
					theta = -(curTheta-Math.PI);
				}
			}
		} 
		
		// If there is no change in y, only travel in the x direction
		else if(diffY == 0)
		{
			// Travel in the positive x direction
			if(diffX > 0)
			{
				if(curTheta >= 0 && curTheta <= Math.PI/2)
				{
					theta = Math.PI/2-curTheta;
				}
				else if(curTheta >= 3*Math.PI/2 && curTheta <= 2*Math.PI)
				{
					theta = 2*Math.PI-curTheta+Math.PI/2;
				}
				else if(curTheta > Math.PI/2 && curTheta < 3*Math.PI/2)
				{
					theta = Math.PI/2-curTheta;
				}

			}
			
			// Travel in the negative x direction
			else if (diffX < 0)
			{
				if(curTheta >= 0 && curTheta <= Math.PI/2)
				{
					theta = -(curTheta+Math.PI/2);
					
				}
				else if(curTheta >= 3*Math.PI/2 && curTheta <= 2*Math.PI)
				{
					theta = -(Math.PI/2-(2*Math.PI-curTheta));
				}
				else if(curTheta > Math.PI/2 && curTheta < 3*Math.PI/2)
				{
					theta = 3*Math.PI/2-curTheta;
				}
			}
		} 
		
		return theta;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	public double[] calculateLineToPathFunction()
	{
		double[] function = new double[2];
		if(Math.abs(xDestOnCurrentPath - o.getX())<1)
		{
			//Notifies that there is no change in x and the robot must go back to x pos
			function[0]=10000;
			
		}
		else if(Math.abs(yDestOnCurrentPath - o.getY())<1)
		{
			//Notifies that there is no change in x and the robot must go back to x pos
			function[1]=10000;
		}
		else
		{
			double a = (yDestOnCurrentPath-o.getY())/(xDestOnCurrentPath-o.getX());
			double b = o.getY()-a*o.getX();
			function[0] = a;
			function[1] = b;
		}
		
		return function;
	}
	
	public boolean functionIsBackOnLineToPath(double[] function, double x, double y)
	{
		if(function[0] == 10000 && (o.getX() >= xDestOnCurrentPath-.1 && o.getX() <= xDestOnCurrentPath+.1))
		{
			return true;
		}
		else if(function[1] == 10000 && (o.getY() >= yDestOnCurrentPath-.1 && o.getY() <= yDestOnCurrentPath+.1))
		{
			return true;
		}
		else if(Math.abs(y - (function[0]*x+function[1])) <= 10)
		{
			return true;
		}
		
		return false;
	}
	
	/*class ObstacleAvoidance extends Thread{  
		public void run(){  
			boolean Continue = true;
			int dist = 0;
			
			while(Continue)
			{
				dist = usPoller.getDistance();
				if(dist <= 17)
				{
					curThread.interrupt();
					double[] function = calculateLineToPathFunction();
					p.setProcessData(true);
					sensorMotor.setSpeed(50);
					sensorMotor.rotate(45);
					//double thetaDifference = Math.abs(initialTheta-o.getTheta());
					try{Thread.sleep(2000);}catch(Exception e){};
					boolean onLineToPath = false;
					while(!onLineToPath)
						{
							onLineToPath = functionIsBackOnLineToPath(function,o.getX(),o.getY());
						}
					p.setProcessData(false);
					sensorMotor.rotate(-45,false);
					usPoller.interrupt();
					travelTo(60,0);
					
					Continue = false;
				}
			}
}

		
}*/
	
}
