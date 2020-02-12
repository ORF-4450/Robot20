package Team4450.Robot20;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pickup extends SubSystem
{
	private Robot		robot;
	private boolean		extended = false, pickupRunning = false;
	
	// This variable used to make this class is a singleton.
	
	private static Pickup 	INSTANCE = null;
	
	private Pickup (Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		Util.consoleLog("Pickup created!");
	}
		
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/	
	public static Pickup getInstance(Robot robot)
	{
		if (INSTANCE == null) INSTANCE = new Pickup(robot);
		
		return INSTANCE;		
	}

	@Override
	void enable()
	{
		Util.consoleLog();
		
		stop();
		
		retract();
	}

	@Override
	void disable()
	{
		Util.consoleLog();

		stop();
		
		retract();
	}

	@Override
	void dispose()
	{
		Util.consoleLog();

		disable();
		
		INSTANCE = null;
	}

	@Override
	protected void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Pickup", pickupRunning);
		SmartDashboard.putBoolean("Pickup Extended", extended);
	}
	
	public void extend()
	{
		Util.consoleLog();
		
		Devices.pickupValve.SetA();
		
		extended = true;
		
		updateDS();
	}
	
	public void retract()
	{
		Util.consoleLog();

		Devices.pickupValve.SetB();
		
		extended = false;
		
		updateDS();
	}
	
	public void start()
	{
		Util.consoleLog();
		
		Devices.pickupTalon.set(.50);
		
		pickupRunning = true;
		
		updateDS();
	}

	public void stop()
	{
		Util.consoleLog();
		
		Devices.pickupTalon.stopMotor();
	
		pickupRunning = false;
	
		updateDS();
	}
	
	public boolean isExtended()
	{
		return extended;
	}
	
	public boolean isPickupRunning()
	{
		return pickupRunning;
	}

}
