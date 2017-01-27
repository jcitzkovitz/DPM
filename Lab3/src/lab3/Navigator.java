package lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

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
	
	public Navigator(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			double leftRadius, double rightRadius, double width, boolean navigate)
			{
				this.leftMotor = leftMotor;
				this.rightMotor = rightMotor;
				this.leftRadius = leftRadius;
				this.rightRadius = rightRadius;
				this.width = width;
				this.navigate = navigate;
				this.o = new Odometer(this.leftMotor,this.rightMotor);
			}
	
	public void drive() {
		// reset the motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { this.leftMotor, this.rightMotor }) {
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
			// Turn from (0,0) to (60,30)
			double theta = calcTurn(0,0,60,30);
			turnTo(theta);
			//Drive from (0,0) to (60,30)
			travelTo(60,30);
			
			// Turn from (60,30) to (30,30)
			theta = calcTurn(this.o.getX(),this.o.getY(),30,30);
			turnTo(theta);		
			//Drive from (60,30) to (30,30)
			travelTo(30,30);
			
			// Turn from (30,30) to (30,60)
			theta = calcTurn(this.o.getX(),this.o.getY(),30,60);
			turnTo(theta);		
			//Drive from (30,30) to (30,60)
			travelTo(30,60);
						
			// Turn from (30,60) to (60,0)
			theta = calcTurn(this.o.getX(),this.o.getY(),60,0);
			turnTo(theta);					
			//Drive from (30,60) to (60,0)
			travelTo(60,0);
		}
	}
	
	public void travelTo(double x, double y)
	{
		double xi = this.o.getX();
		double yi = this.o.getY();
		double dist = Math.sqrt(Math.pow(x-xi,2)+Math.pow(y-yi,2));
		this.leftMotor.setSpeed(FORWARD_SPEED);
		this.rightMotor.setSpeed(FORWARD_SPEED);
		this.leftMotor.rotate(convertDistance(leftRadius, dist), true);
		this.rightMotor.rotate(convertDistance(rightRadius, dist), false);
	}
	
	public void turnTo(double theta)
	{
		this.leftMotor.setSpeed(ROTATE_SPEED);
		this.rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(leftRadius, width, theta), true);
		rightMotor.rotate(-convertAngle(rightRadius, width, theta), false);
	}
	
	public boolean isNavigating()
	{
		return true;
	}
	
	
	public double calcTurn(double xi, double yi, double xf, double yf)
	{
		double diffX = xf-xi;
		double diffY = yf-yi;
		double theta = 0;
		double curTheta = this.o.getTheta();
		double axisTheta = 0;
		
		
		if(diffX > 0 && diffY > 0)			// Traveling to positive x and y
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = 90-theta;
			
			double theta1 = 360-curTheta+axisTheta;
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
		else if(diffX > 0 && diffY < 0)			// Traveling to negative y and positive x
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = 90-theta;
			
			double theta1 = 360-curTheta+axisTheta;
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
		
		else if(diffX < 0 && diffY < 0)
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = 270-theta;
			
			double theta1 = 360-curTheta+axisTheta;
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

		else if(diffX < 0 && diffY > 0)
		{
			theta = Math.atan2(diffY,diffX);
			axisTheta = 270-theta;
			
			double theta1 = 360-curTheta+axisTheta;
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
		
		else if(diffX == 0)
		{
			if(diffY > 0)
			{
				if(curTheta >= 0 && curTheta <= 180)
				{
					theta = -curTheta;
				}
				else if(curTheta > 180 && curTheta <= 360)
				{
					theta = 360-curTheta;
				}

			}
			else if (diffY < 0)
			{
				if(curTheta >= 0 && curTheta <= 180)
				{
					theta = 180-curTheta;
				}
				else if(curTheta > 180 && curTheta <= 360)
				{
					theta = -(curTheta-180);
				}
			}
		} 
		else if(diffY == 0)
		{
			if(diffX > 0)
			{
				if(curTheta >= 0 && curTheta <= 90)
				{
					theta = 90-curTheta;
				}
				else if(curTheta >= 270 && curTheta <= 360)
				{
					theta = 360-curTheta+90;
				}
				else if(curTheta > 90 && curTheta < 270)
				{
					theta = 90-curTheta;
				}

			}
			else if (diffX < 0)
			{
				if(curTheta >= 0 && curTheta <= 90)
				{
					theta = -(curTheta+90);
				}
				else if(curTheta >= 270 && curTheta <= 360)
				{
					theta = -(360-curTheta+90);
				}
				else if(curTheta > 90 && curTheta < 270)
				{
					theta = 180-curTheta;
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
	
}
