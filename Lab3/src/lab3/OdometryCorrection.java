/* 
 * OdometryCorrection.java
 */
package lab3;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;

	
	//public static Port sensorPort = LocalEV3.get().getPort("S1");   
	public static Port portColor = LocalEV3.get().getPort("S1");
	public static SensorModes myColor = new EV3ColorSensor(portColor);
	public static SampleProvider myColorSample = myColor.getMode("Red");
	public static float[] sampleColor = new float[myColor.sampleSize()];
	public static int numSamples = 0;
	public static int count = 0;
	boolean testLine = false;
	double priorTHETA = 0;
	double deltaTHETA = 0;
	double THETA = 0;
	
	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		
		double yDist = 0; //temp value for distance along y axis
		double xDist = 0; // temp value for distance along x axis


		while (true) {
			correctionStart = System.currentTimeMillis();

			// put your correction code here
			
			myColorSample.fetchSample(sampleColor,0);
			
			// If the robot crosses a black line, mark testLine as true
			
			if ( (sampleColor[0]*1000) < 200) {
				testLine = true;
				
				
			}
			
			// If the line is there increment the amount of lines crossed and make the correction
			// Note: Sensor is 3.8cm ahead from  the wheel base
			double sensorDiff = 3.8;
			
			if ((sampleColor[0]*1000) > 200 && testLine == true){
				
				testLine = false;
				count++;
				
				// ******* Y Correction *******
				
				// There will not be much error in the y direction prior to the first line crossing in the y 
				// direction, thus use the odometer reading as a reference point
				// Y direction traveled (when travelling in the Y) will be yDist*cos(Theta) and X direction
				// travelled will be xDist*sin(Theta)
				
				THETA = this.odometer.getTheta();
				deltaTHETA = THETA-priorTHETA;
				
				if(count == 1)
				{
					yDist = odometer.getY();
					xDist = odometer.getX();
					priorTHETA = this.odometer.getTheta();
				}
				else if (count >= 2 && count < 4)
				{
					yDist+=30.48*Math.cos(deltaTHETA);
					odometer.setY(yDist);
					xDist+=30.48*Math.sin(deltaTHETA);
					odometer.setX(xDist);
					priorTHETA = this.odometer.getTheta();
				}
				
				// Reset the reference y value to the odometers current reading
				
				else if(count == 7)
				{
					yDist = odometer.getY();
					xDist = odometer.getX();
					priorTHETA = this.odometer.getTheta();
				}
				
				else if(count >=8 && count < 10)
				{
					yDist-=30.48*Math.cos(deltaTHETA);
					odometer.setY(yDist);
					xDist+=30.48*Math.sin(deltaTHETA);
					odometer.setX(xDist);
					priorTHETA = this.odometer.getTheta();
				}
				// ***** X Correction ******
				
				// There will not be much error in the x correction prior to the first line crossing in
				// the x direction, thus use the odometer reading as a reference point
				// X direction traveled (when travelling in the X) will be xDist*cos(Theta) and Y direction
				// travelled will be yDist*sin(Theta)
				
				else if(count == 4)
				{
					xDist = odometer.getX();
					yDist = odometer.getY();
					priorTHETA = this.odometer.getTheta();
				}
				else if(count > 4 && count < 7)
				{
					xDist+=30.48*Math.cos(deltaTHETA);
					odometer.setX(xDist);
					yDist+=30.48*Math.sin(deltaTHETA);
					odometer.setY(yDist);
					priorTHETA = this.odometer.getTheta();
				}
				
				// Reset the reference x value to the odometers current reading
				
				else if(count == 10)
				{
					xDist = odometer.getX();
					yDist = odometer.getY();
					priorTHETA = this.odometer.getTheta();
				}
				
				else if(count >= 11 && count < 13)
				{
					xDist-=30.48*Math.cos(deltaTHETA);
					odometer.setX(xDist);
					yDist+=30.48*Math.sin(deltaTHETA);
					odometer.setY(yDist);
					priorTHETA = this.odometer.getTheta();
				}
				
				
				
				Sound.beep();
			}
			
			
			numSamples++;
			
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
}