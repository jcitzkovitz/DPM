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
				
				// Set local constants
				armMotor.setAcceleration(3000);
				leftMotor.setSpeed(100);
				rightMotor.setSpeed(100);
				int armRotAngle = 140;
				int sleepLong = 2000;
				int sleepShort = 1000;
				int robotRotateAngle = 20;
				int rotateArmSpeedLow = 200;
				
				// Create object for drawing on screen
				final TextLCD t = LocalEV3.get().getTextLCD();
				
				// clear the display
				t.clear();

				// ask the user whether the motors should drive in a square or float
				t.drawString("< Left: Shoot Left", 0, 0);
				t.drawString("> Right: Shoot Right", 0, 1);
				t.drawString("^ Up: Shoot Ahead", 0, 2);
				
				int buttonChoice;
				buttonChoice = Button.waitForAnyPress();
					
				/* While the escape button has not been pressed, the robot will be able to shoot balles
				 * either straight ahead, to the right or to the left without exiting the program*/
				while (Button.waitForAnyPress() != Button.ID_ESCAPE)
				{
					buttonChoice = Button.waitForAnyPress();
					
					/* Shoot Left */
					if (buttonChoice == Button.ID_LEFT) {				
						
						// Sleep the robot for two seconds in order for humans to not interfere with motion
						sleepThread(sleepLong);
						
						// Rotate the robot clockwise
						rotateRobot(robotRotateAngle,true);
						sleepThread(sleepShort);
						
						// Rotate the arm and shoot the ball at the left target
						rotateArm(armRotAngle,armMotor.getMaxSpeed());
						
						// Return the arm slowly to ensure as little backward momentum as possible
						rotateArm(-armRotAngle,rotateArmSpeedLow);
						rotateRobot(robotRotateAngle,false);
					
					}
					
					/* Shoot Ahead */
					else if(buttonChoice == Button.ID_UP)
					{
						sleepThread(sleepLong);
						
						// Rotate the arm and shoot the ball at the target straight ahead
						rotateArm(armRotAngle,armMotor.getMaxSpeed());
						rotateArm(-armRotAngle,rotateArmSpeedLow);
					} 
					
					/* Shoot Right */
					else
					{
						sleepThread(sleepLong);
						
						// Rotate the robot counterclockwise
						rotateRobot(robotRotateAngle,false);
						sleepThread(sleepShort);
						
						// Rotate the arm and shoot the ball at the right target
						rotateArm(armRotAngle,armMotor.getMaxSpeed());
						rotateArm(-armRotAngle,rotateArmSpeedLow);
						rotateRobot(robotRotateAngle,true);
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
			
			/* Rotate the robot either left (true boolean value) or right (false boolean value) */
			public static void rotateRobot(int angle, boolean turnLeft)
			{
				if(turnLeft)
				{
					leftMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,-angle),true);
					rightMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,angle),false);
				}
				else
				{
					leftMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,angle),true);
					rightMotor.rotate(convertAngle(WHEEL_RADIUS,TRACK,-angle),false);
				}
			}
			
			/* Rotate the arm in order to shoot the ball */
			public static void rotateArm(int angle, int speed)
			{
				armMotor.setSpeed(speed);
				armMotor.rotate(angle);
			}
			
			/* Sleep the thread */
			public static void sleepThread(int timeInMillis)
			{
				try{Thread.sleep(timeInMillis);}catch(Exception e){};
			}
	
}
