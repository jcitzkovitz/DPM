package lab3;

import lejos.hardware.Button;   
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lab3.Odometer;
import lab3.OdometryCorrection;
import lab3.OdometryDisplay;
import lab3.Navigator;

public class Lab3 {

	// Static Resources:
		// Left motor connected to output A
		// Right motor connected to output D
		private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

		// Setup port for ultrasonic sensor
		private static final Port usPort = LocalEV3.get().getPort("S1");
		
		// Constants
		public static final double WHEEL_RADIUS = 2.095;
		public static final double TRACK = 16.35;
		private static final int bandCenter = 30;			// Offset from the wall (cm)
		private static final int bandWidth =3;				// Width of dead band (cm)

		public static void main(String[] args) {
			int buttonChoice;

			// Some objects that need to be instantiated
			final TextLCD t = LocalEV3.get().getTextLCD();
			final Odometer odometer = new Odometer(leftMotor, rightMotor);
			OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
			
			do {
				// Clear the display
				t.clear();

				// Ask the user whether the motors should drive in a square or float
				t.drawString("< Left    | Right >", 0, 0);
				t.drawString("          |        ", 0, 1);
				t.drawString(" Navigate | Drive  ", 0, 2);
				t.drawString("          | on   ", 0, 3);
				t.drawString("          | path ", 0, 4);

				buttonChoice = Button.waitForAnyPress();
			} while (buttonChoice != Button.ID_LEFT
					&& buttonChoice != Button.ID_RIGHT);
			
			// Setup ultrasonic sensor
			@SuppressWarnings("resource")							    
			final SensorModes usSensor = new EV3UltrasonicSensor(usPort);
			final SampleProvider usDistance = usSensor.getMode("Distance");	
			float[] usData = new float[usDistance.sampleSize()];		
		
			// Instantiate p controller object
			final PController p = new PController(leftMotor, rightMotor, bandCenter, bandWidth);
			
			// Instantiate UltrasonicPoller thread
			final UltrasonicPoller usPoller = new UltrasonicPoller(usDistance, usData, p);
			
			if (buttonChoice == Button.ID_LEFT) {				
				
				// Start the odometer, odometerDisplay and the usPoller threads
				odometer.start();
				odometryDisplay.start();
				usPoller.start();
				
				// Spawn a new Thread to avoid Navigator.drive() from blocking
				(new Thread() {
					public void run() {
						Navigator nav1 = new Navigator(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, true, odometer,usPoller,Thread.currentThread(),p);
						nav1.drive();
					}
				}).start();
				
			} else {
				
				// Start the odometer, the odometry display and (possibly) the
				odometer.start();
				odometryDisplay.start();

				// spawn a new Thread to avoid Navigator.drive() from blocking
				(new Thread() {
					public void run() {
						Navigator nav2 = new Navigator(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, false, odometer, usPoller, Thread.currentThread(),p);
						nav2.drive();
					}
				}).start();
			}
			
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
		}
	}
