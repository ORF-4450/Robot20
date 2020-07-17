
package Team4450.Robot20;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import Team4450.Lib.NavX.NavXEvent;
import Team4450.Lib.NavX.NavXEventListener;
import Team4450.Lib.NavX.NavXEventType;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Lib.Wpilib.PIDController;
import Team4450.Lib.Wpilib.PIDSourceType;
import Team4450.Robot20.Devices;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Teleop
{
	private final Robot 		robot;
	private boolean				autoTarget, altDriveMode;

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
		String	gameData = "";
		int		angle;
		Pose2d	pose = Devices.odometer.getPose();
		
		SynchronousPID	sPid = new SynchronousPID(1, 0 , 0);
		sPid.setName("syncPIDTest");
		
		PIDSourceShim	pidSource = new PIDSourceShim(null);
		pidSource.setPIDSourceType(PIDSourceType.kDisplacement);
		
		PIDOutputShim	pidOutput = new PIDOutputShim(null);
		
		PIDController	pid = new PIDController(1, 0, 0, pidSource, pidOutput);
		pid.setName("PIDtest");
		pidSource.setPidController(pid);

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
		
		Devices.utilityStick.deadZoneY(.25);
		Devices.utilityStick.deadZoneX(.25);

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
		Devices.climber.enable();
		Devices.pickup.enable();
//		Devices.shooter.enable();
//		Devices.channel.enable();
		
		sPid.setOutputRange(-1, 1);
		sPid.setSetpoint(1.0);
		SmartDashboard.putData(sPid.getName(), sPid);
		
		pid.setOutputRange(-1, 1);
		pid.setSetpoint(1.0);
		pid.setEnabled(true);
		SmartDashboard.putData(pid.getName(), pid);

		// Motor safety turned on.
		Devices.robotDrive.setSafetyEnabled(true);

		// Driving loop runs until teleop is over.

		Util.consoleLog("enter driving loop");
		
		LaunchPadControl rocker = Devices.launchPad.FindButton(LaunchPadControlIDs.ROCKER_LEFT_BACK);

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
			utilX = Devices.utilityStick.GetX();
			
			sPid.calculate(utilX, .02);
			//sPid.updateSendable();
			pidSource.set(utilX);
			
			LCD.printLine(2, "leftenc=%d  rightenc=%d", Devices.leftEncoder.get(), Devices.rightEncoder.get());			
			LCD.printLine(3, "leftY=%.3f (%.3f)  rightY=%.3f (%.3f)  -  rightX=%.3f  utilY=%.3f  utilX=%.3f", leftY, 
					 Devices.LRCanTalon.get(), rightY, Devices.RRCanTalon.get(), rightX, utilY, utilX);
			LCD.printLine(4, "yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), 
					Devices.navx.getTotalYaw(), Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(5, "winchSwitch=%b  winchEnc=%d  ballEye=%b  rocker=%b", Devices.winchSwitch.get(),
					Devices.winchEncoder.get(), !Devices.ballEye.get(), rocker.currentState);
			LCD.printLine(6, "pose x=%.1f  y=%.1f  deg=%.1f", pose.getTranslation().getX(), pose.getTranslation().getY(),
							pose.getRotation().getDegrees());
			LCD.printLine(7, "Lmax vel=%.3f - Rmax vel=%.3f", Devices.leftEncoder.getMaxVelocity(PIDRateType.velocityMPS),
					 Devices.rightEncoder.getMaxVelocity(PIDRateType.velocityMPS));
			//LCD.printLine(7, "shooter rpm=%d", Devices.shooterEncoder.getRPM());
			LCD.printLine(8, "rate=%d  maxrate=%d  rpm=%d  maxrpm=%d  vel=%.3f  maxvel=%.3f", 
					Devices.leftEncoder.getRate(PIDRateType.ticksPer100ms), 
					Devices.leftEncoder.getMaxRate(PIDRateType.ticksPer100ms),
					Devices.leftEncoder.getRPM(),
					Devices.leftEncoder.getMaxRPM(),
					Devices.leftEncoder.getVelocity(PIDRateType.velocityMPS),
					Devices.leftEncoder.getMaxVelocity(PIDRateType.velocityMPS)
					);
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
						Devices.robotDrive.tankDrive(leftY, rightY, true);	// Normal tank drive.
					}

					SmartDashboard.putBoolean("SteeringAssist", steeringAssistMode);
				}
				else
					Devices.robotDrive.tankDrive(leftY, rightY, true);		// Normal tank drive.
				
					// This shows how to use curvature drive mode, toggled by trigger (for testing).
					//Devices.robotDrive.curvatureDrive(rightY, rightX, rightStick.GetLatchedState(JoyStickButtonIDs.TRIGGER));
			}

			// Control climb winch with utility stick, pull back is up (-).
			
			Devices.climber.set(Util.squareInput(utilY));
			
			//Devices.hookVictor.set(utilX);
			
			pose = Devices.odometer.update();
			
			if (firsttime) Util.consoleLog("after first loop");
			
			firsttime = false;

			// Cause smartdashboard to update any registered Sendables, including Gyro2.
			
			SmartDashboard.updateValues();
			
			// Update game color on DS.

			gameData = "R" ; //DriverStation.getInstance().getGameSpecificMessage();
			
			if (gameData != null) SmartDashboard.putString("GameColor", ColorWheel.convertGameColor(gameData));


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
					
				// Start color wheel for manual operation.
				case BUTTON_BLUE:
					if (Devices.colorWheel.isRunning())
						Devices.colorWheel.stopWheel();
					else
						Devices.colorWheel.startWheel(.25);
					
					break;
					
				// Start color wheel for counted turns.
				case BUTTON_BLUE_RIGHT:
					if (Devices.colorWheel.isRunning())
						Devices.colorWheel.stopWheel();
					else
						Devices.colorWheel.startWheel(.25, false);
					
					break;
					
				// Start color wheel for game color.
				case BUTTON_YELLOW:
					if (Devices.colorWheel.isRunning())
						Devices.colorWheel.stopWheel();
					else
						Devices.colorWheel.startWheel(.25, true);
					
					break;
					
				case BUTTON_RED_RIGHT:
					if (Devices.climber.isBrakeEngaged())
						Devices.climber.releaseBrake();
					else
						Devices.climber.engageBrake();
					
					break;
					
				case BUTTON_BLACK:
					//Devices.pickup.startBallDetector();
					//Devices.pickup.enableBalldetector();
					Devices.ballEye.enableInterrupts();
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
					
				// Move ball toward shooter.
				case TOP_LEFT:
					if (Devices.channel.isRunning())
						Devices.channel.stop();
					else
						Devices.channel.start(.70);
					
					break;
					
				// Move ball backwards toward pickup.
				case TOP_RIGHT:
					if (Devices.channel.isRunning())
						Devices.channel.stop();
					else
						Devices.channel.start(-.70);
					
					break;
					
				case TOP_BACK:
					if (Devices.pickup.isExtended())
						Devices.pickup.retract();
					else
						Devices.pickup.extend();
					
					break;
					
				// Shooter motor toggle.
				case TOP_MIDDLE:
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
