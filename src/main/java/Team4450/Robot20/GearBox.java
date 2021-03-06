/**
 * Manage gear box shifting.
 */

package Team4450.Robot20;

import Team4450.Lib.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearBox extends SubSystem
{
	private Robot			robot;
	private boolean			lowSpeed,  highSpeed;
	
	// This variable used to make this class is a singleton.
	
	private static GearBox 	gearBox = null;
	
	private GearBox (Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		//lowSpeed();
		
		Util.consoleLog("GearBox created!");
	}
		
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/	
	public static GearBox getInstance(Robot robot)
	{
		if (gearBox == null) gearBox = new GearBox(robot);
		
		return gearBox;		
	}

	/*
	 * Called when robot enabled.
	 */
	void enable()
	{
		Util.consoleLog();
		
		lowSpeed();
	}

	/*
	 * Called when robot disabled.
	 */
	void disable()
	{
		Util.consoleLog();
		
	}

	/**
	 * Release any resources and the shared instance of this class.
	 */
	public void dispose()
	{
		Util.consoleLog();
		
		gearBox = null;
	}
	
	protected void updateDS()
	{
		Util.consoleLog("low=%b, high=%b", lowSpeed, highSpeed);
		
		SmartDashboard.putBoolean("Low", lowSpeed);
		SmartDashboard.putBoolean("High", highSpeed);
	}

	/**
	 * Set gear boxes into low speed. Pushes the dog ring to the inside.
	 * Brake mode off.
	 */
	public void lowSpeed()
	{
		Util.consoleLog();

		highSpeed = false;

		Devices.highLowValve.SetA();
		
		//Devices.SetCANTalonBrakeMode(false);
		
		lowSpeed = true;
		
		updateDS();
	}

	/**
	 * Set gear boxes into high speed. Pushes the dog ring to the outside.
	 * Brake mode on.
	 */
	public void highSpeed()
	{
		Util.consoleLog();

		lowSpeed = false;
		
		Devices.highLowValve.SetB();
		
		//Devices.SetCANTalonBrakeMode(true);
		
		highSpeed = true;
		
		updateDS();
	}

	/**
	 * Return low speed state.
	 * @return True if low speed.
	 */
	public boolean isLowSpeed()
	{
		return lowSpeed;
	}
	
	/**
	 * Return high speed state.
	 * @return True if high speed.
	 */
	public boolean isHighSpeed()
	{
		return highSpeed;
	}
}
