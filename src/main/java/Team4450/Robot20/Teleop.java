
package Team4450.Robot20;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import Team4450.Lib.NavX.NavXEvent;
import Team4450.Lib.NavX.NavXEventListener;
import Team4450.Lib.NavX.NavXEventType;
import Team4450.Robot20.Devices;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatchResult;

class Teleop
{
	private final Robot 		robot;
	private boolean				autoTarget, altDriveMode;
	
  	RevColorSensor				colorSensor = RevColorSensor.getInstance();

	// This variable used to make this class is a singleton.
	
	private static Teleop 		teleop = null;
	
	// Private constructor prevents multiple instances from being created.

	private Teleop(Robot robot)
	{
		Util.consoleLog();
		
		// Motor safety turned off during initialization.
		//Devices.robotDrive.setSafetyEnabled(false);

		this.robot = robot;
	}
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/
	public static Teleop getInstance(Robot robot) 
	{
		if (teleop == null) teleop = new Teleop(robot);
		
		return teleop;
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	void dispose()
	{
		Util.consoleLog();

		Devices.launchPad.removeAllLaunchPadEventListeners();
		Devices.leftStick.removeAllJoyStickEventListeners();
		Devices.rightStick.removeAllJoyStickEventListeners();
		Devices.utilityStick.removeAllJoyStickEventListeners();

		teleop = null;
	}

	void OperatorControl() throws Exception
	{
		double	rightY = 0, leftY = 0, utilX = 0, utilY = 0, rightX = 0, leftX = 0;
		double	gain = .05;
		boolean	steeringAssistMode = false;
		boolean firsttime = true;
		int		angle;

		// Motor safety turned off during initialization.

		Util.consoleLog();

		LCD.printLine(1, "Mode: teleop All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, Devices.ds.isFMSAttached());
		
		// Configure LaunchPad and Joystick event handlers.

		Devices.launchPad.addLaunchPadEventListener(new LaunchPadListener());

		Devices.leftStick.addJoyStickEventListener(new LeftStickListener());

		Devices.rightStick.addJoyStickEventListener(new RightStickListener());
		
		Devices.utilityStick.addJoyStickEventListener(new UtilityStickListener());
		
		// Invert driving joy stick Y axis so + values mean f.
		Devices.leftStick.invertY(true);
		Devices.rightStick.invertY(true);
		
		Devices.utilityStick.deadZoneY(.20);

		// 2018 post season testing showed Anakin liked this setting, smoothing driving.
		Devices.SetCANTalonRampRate(0.5);
		
		// Set Navx current yaw to 0.
		Devices.navx.resetYaw();

		// Reset wheel encoders.
		Devices.leftEncoder.reset();
		Devices.rightEncoder.reset();
		
		// Reset climber gyro.
		
		//Devices.gyro.reset();
		
		// Put subsystem objects into start up state.
		Devices.gearBox.enable();
//		Devices.climber.enable();
//		Devices.pickup.enable();
//		Devices.shooter.enable();
//		Devices.channel.enable();

		// Motor safety turned on.
		Devices.robotDrive.setSafetyEnabled(true);

		// Driving loop runs until teleop is over.

		setTargetColor();
		
		Util.consoleLog("enter driving loop");

		while (robot.isEnabled())	// && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object
			// using calls to our JoyStick class.

			//rightY = stickLogCorrection(rightStick.GetY());	// fwd/back
			//leftY = stickLogCorrection(leftStick.GetY());	// fwd/back

			//rightX = stickLogCorrection(rightStick.GetX());	// left/right
			//leftX = stickLogCorrection(leftStick.GetX());	// left/right
			
			rightY = Devices.rightStick.GetY();	// fwd/back
			leftY = Devices.leftStick.GetY();	// fwd/back

			rightX = Devices.rightStick.GetX();	// left/right
			leftX = Devices.leftStick.GetX();	// left/right

			utilY = Devices.utilityStick.GetY();
			
			LCD.printLine(2, "leftenc=%d  rightenc=%d", Devices.leftEncoder.get(), Devices.rightEncoder.get());			
			LCD.printLine(3, "leftY=%.3f (%.3f)  rightY=%.3f (%.3f)  -  rightX=%.3f  utilY=%.3f", leftY, 
					 Devices.LRCanTalon.get(), rightY, Devices.RRCanTalon.get(), rightX, utilY);
			LCD.printLine(4, "yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), 
					Devices.navx.getTotalYaw(), Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(5, "color match=%b", colorMatch());
			//LCD.printLine(7, "shooter rpm=%d", Devices.shooterEncoder.getRPM());
			//LCD.printLine(6, "gyro angle=%f  center=%d  offset=%f", Devices.gyro.getAngle(), Devices.gyro.getCenter(), Devices.gyro.getOffset());

			//LCD.printLine(10, "pressureV=%.2f  psi=%d  ustb=%b", robot.monitorCompressorThread.getVoltage(), 
			//		robot.monitorCompressorThread.getPressure(), Devices.utilityStick.GetCurrentState(JoyStickButtonIDs.TOP_BACK));
			
			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			// Two drive modes, full tank and alternate. Switch on right stick trigger.

			if (!autoTarget) 
			{
				if (altDriveMode)
				{	// normal tank with straight drive assist when sticks within 10% of each other and
					// right stick power is greater than 50%.
					if (isLeftRightEqual(leftY, rightY, 10) && Math.abs(rightY) > .50)
					{
						// Reset angle measurement when entering this code first time after mode is enabled.
						if (!steeringAssistMode) Devices.navx.resetYaw();

						// Angle is negative if robot veering left, positive if veering right when going forward.
						// It is opposite when going backward.

						angle = (int) Devices.navx.getYaw();

						LCD.printLine(5, "angle=%d", angle);

						// Invert angle for backwards movement.

						if (rightY < 0) angle = -angle;

						//Util.consoleLog("angle=%d", angle);

						// Note we invert sign on the angle because we want the robot to turn in the opposite
						// direction than it is currently going to correct it. So a + angle says robot is veering
						// right so we set the turn value to - because - is a turn left which corrects our right
						// drift.
						
						Devices.robotDrive.curvatureDrive(rightY, -angle * gain, false);

						steeringAssistMode = true;
					}
					else
					{
						steeringAssistMode = false;
						Devices.robotDrive.tankDrive(leftY, rightY);	// Normal tank drive.
					}

					SmartDashboard.putBoolean("SteeringAssist", steeringAssistMode);
				}
				else
					Devices.robotDrive.tankDrive(leftY, rightY);		// Normal tank drive.
				
					// This shows how to use curvature drive mode, toggled by trigger (for testing).
					//Devices.robotDrive.curvatureDrive(rightY, rightX, rightStick.GetLatchedState(JoyStickButtonIDs.TRIGGER));
			}

			//Devices.shooterTalon.set(utilY);
			
			if (firsttime) Util.consoleLog("after first loop");
			
			firsttime = false;
		
			// Cause smartdashboard to update any registered Sendables, including Gyro2.
			
			SmartDashboard.updateValues();
			
			// End of driving loop.

			Timer.delay(.020);	// wait 20ms for update from driver station.
		}

		// End of teleop mode.

		Util.consoleLog("end");
	}

	private boolean isLeftRightEqual(double left, double right, double percent)
	{
		if (Math.abs(left - right) <= (1 * (percent / 100))) return true;

		return false;
	}

	// Custom base logarithm.
	// Returns logarithm base of the value.

	private double baseLog(double base, double value)
	{
		return Math.log(value) / Math.log(base);
	}

	// Map joystick y value of 0.0 to 1.0 to the motor working power range of approx 0.5 to 1.0 using
	// logarithmic curve.

	private double stickLogCorrection(double joystickValue)
	{
		double base = Math.pow(2, 1/3) + Math.pow(2, 1/3);

		if (joystickValue > 0)
			joystickValue = baseLog(base, joystickValue + 1);
		else if (joystickValue < 0)
			joystickValue = -baseLog(base, -joystickValue + 1);

		return joystickValue;
	}
	
	// Reads game data and if color code present, loads the correct Color
	// object into the Color Sensor matching function.
	
	private void setTargetColor()
	{
		Color	targetColor = null;
		String 	gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		gameData = "R";
		
		Util.consoleLog(gameData);
		
		if(gameData.length() > 0)
		{
			switch (gameData.charAt(0))
			{
				case 'B' :
					//Blue code.
					targetColor = RevColorSensor.getMatchColor(0.143, 0.427, 0.429);
					break;
					
				case 'G' :
					//Green code.
					targetColor = RevColorSensor.getMatchColor(0.197, 0.561, 0.240);
					break;
					
				case 'R' :
					//Red code.
					targetColor = RevColorSensor.getMatchColor(0.561, 0.232, 0.114);
					break;
					
				case 'Y' :
					//Yellow code.
					targetColor = RevColorSensor.getMatchColor(0.361, 0.524, 0.113);
					break;
					
				default :
					//This is corrupt data.
					break;
		  }
		} else {
			// no data received yet default to red.
			targetColor = RevColorSensor.getMatchColor(0.561, 0.232, 0.114);
		}		
		
		if (targetColor != null) colorSensor.addColorMatch(targetColor);
	}
	
	// Match current color read from sensor to the target color in the color matcher
	// and returns true if current color matches target color with confidence
	// >= 85%.
	
	private boolean colorMatch()
	{
		Color color = colorSensor.getColor();

		ColorMatchResult matchResult = colorSensor.matchClosestColor(color);

		//LCD.printLine(6, "color match result r=%f g=%f b=%f  conf=%f", matchResult.color.red, matchResult.color.green,
		//		matchResult.color.blue, matchResult.confidence);

		if (matchResult.confidence >= .85)
			return true;
		else
			return false;
	}

	// Handle LaunchPad control events.

	public class LaunchPadListener implements LaunchPadEventListener 
	{
		public void ButtonDown(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s, latchedState=%b ord=%d hc=%d %s", control.id.name(),  control.latchedState, control.id.ordinal(), control.id.hashCode(), control.id.getClass().toString());

			switch(control.id)
			{
				case BUTTON_RED:
					Devices.leftEncoder.reset();
					Devices.rightEncoder.reset();
					break;
					
				default:
					break;
			}
		}

		public void ButtonUp(LaunchPadEvent launchPadEvent) 
		{
			//Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.name(),  launchPadEvent.control.latchedState);
		}

		public void SwitchChange(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s", control.id.name());

			switch(control.id)
			{
				// Set CAN Talon brake mode.
	    		case ROCKER_LEFT_BACK:
    				if (Devices.isBrakeMode())
    					Devices.SetCANTalonBrakeMode(false);	// coast
    				else
    					Devices.SetCANTalonBrakeMode(true);		// brake
    				
    				break;
    				
	    		case ROCKER_LEFT_FRONT:
					if (robot.cameraThread != null) robot.cameraThread.ChangeCamera();
	    			break;
	    			
				default:
					break;
			}
		}
	}

	// Handle Right JoyStick Button events.

	private class RightStickListener implements JoyStickEventListener 
	{

		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					altDriveMode = !altDriveMode;
					SmartDashboard.putBoolean("AltDriveMode", altDriveMode);
					break;

				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Left JoyStick Button events.

	private class LeftStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					if (Devices.gearBox.isLowSpeed())
	    				Devices.gearBox.highSpeed();
	    			else
	    				Devices.gearBox.lowSpeed();

					break;

				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Utility JoyStick Button events.

	private class UtilityStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					break;
					
				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.id.name());
		}
	}
	
	// Handle NavX events.

	private class NavXListener implements NavXEventListener 
	{
		public void event( NavXEvent navXEvent )
		{
			if ( navXEvent.eventType == NavXEventType.collisionDetected)
				Util.consoleLog("collision detected = %3f", navXEvent.eventData);
		}
	}
}
