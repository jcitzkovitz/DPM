package lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
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

		// Constants
		public static final double WHEEL_RADIUS = 2.095;
		public static final double TRACK = 16.35;

		public static void main(String[] args) {
			int buttonChoice;

			// some objects that need to be instantiated
			
			final TextLCD t = LocalEV3.get().getTextLCD();
			Odometer odometer = new Odometer(leftMotor, rightMotor);
			OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
			OdometryCorrection odometryCorrection = new OdometryCorrection(odometer);

			do {
				// clear the display
				t.clear();

				// ask the user whether the motors should drive in a square or float
				t.drawString("< Left    | Right >", 0, 0);
				t.drawString("          |        ", 0, 1);
				t.drawString(" Navigate | Drive  ", 0, 2);
				t.drawString("          | on   ", 0, 3);
				t.drawString("          | path ", 0, 4);

				buttonChoice = Button.waitForAnyPress();
			} while (buttonChoice != Button.ID_LEFT
					&& buttonChoice != Button.ID_RIGHT);

			if (buttonChoice == Button.ID_LEFT) {
				
				odometer.start();
				odometryDisplay.start();
				
				// spawn a new Thread to avoid SquareDriver.drive() from blocking
				(new Thread() {
					public void run() {
						Navigator.drive(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, true);
					}
				}).start();
				
			} else {
				// start the odometer, the odometry display and (possibly) the
				// odometry correction
				
				odometer.start();
				odometryDisplay.start();

				// spawn a new Thread to avoid SquareDriver.drive() from blocking
				(new Thread() {
					public void run() {
						Navigator.drive(leftMotor, rightMotor, WHEEL_RADIUS, WHEEL_RADIUS, TRACK, false);
					}
				}).start();
			}
			
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);
		}
	}