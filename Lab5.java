package lab5;


import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;



public class Lab5 {

	// Static Resources:
			// Left motor connected to output A
			// Right motor connected to output D
			private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
			private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
			private static final EV3LargeRegulatedMotor armMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
			// Setup port for ultrasonic sensor
			private static final Port usPort = LocalEV3.get().getPort("S1");
			
			// Constants
			public static final double WHEEL_RADIUS = 2.095;
			public static final double TRACK = 16.35;
			private static final int bandCenter = 30;			// Offset from the wall (cm)
			private static final int bandWidth =3;
			
			public static void main(String[] args) {
				
				int buttonChoice;
				final TextLCD t = LocalEV3.get().getTextLCD();
				Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
				Navigation nav = new Navigation(odo);
				odo.start();
				buttonChoice = Button.waitForAnyPress();
				
					// clear the display
					t.clear();

					// ask the user whether the motors should drive in a square or float
					t.drawString("< Left |   Up   | Right >", 0, 0);
					t.drawString("       |        |        ", 0, 1);
					t.drawString(" Left  |  Ahead | Right  ", 0, 2);
					t.drawString(" angle |        | angle  ", 0, 3);
					t.drawString("       |        |        ", 0, 4);
			
					leftMotor.setSpeed(100);
					rightMotor.setSpeed(100);
					int armRotAngle = 170;
				while (Button.waitForAnyPress() != Button.ID_ESCAPE)
				{
					buttonChoice = Button.waitForAnyPress();
					if (buttonChoice == Button.ID_LEFT) {				
						
						leftMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,-20),true);
						rightMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,20),false);
						try{Thread.sleep(2000);}catch(Exception e){};
						armMotor.setSpeed(armMotor.getMaxSpeed());
						armMotor.rotate(armRotAngle);
						armMotor.setSpeed(200);
						armMotor.rotate(-armRotAngle);
						leftMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,20),true);
						rightMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,-20),false);
					
					} else if(buttonChoice == Button.ID_UP)
					{
						try{Thread.sleep(1000);}catch(Exception e){};
						armMotor.setSpeed(armMotor.getMaxSpeed());
						armMotor.rotate(armRotAngle);
						armMotor.setSpeed(200);
						armMotor.rotate(-armRotAngle);
					} else
					{
						leftMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,20),true);
						rightMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,-20),false);
						try{Thread.sleep(2000);}catch(Exception e){};
						armMotor.setSpeed(armMotor.getMaxSpeed());
						armMotor.rotate(armRotAngle);
						armMotor.setSpeed(200);
						armMotor.rotate(armRotAngle);
						leftMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,-20),true);
						rightMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,20),false);
					}
				}
				System.exit(0);
			}
	
			private static int convertDistance(double radius, double distance) {
				return (int) ((180.0 * distance) / (Math.PI * radius));
			}

			private static int convertAngle(double radius, double width, double angle) {
				return convertDistance(radius, Math.PI * width * angle / 360.0);
			}
	
}