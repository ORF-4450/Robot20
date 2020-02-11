package Team4450.Robot20;

import Team4450.Lib.Util;

public class Pickup extends SubSystem
{
	private Robot			robot;
	
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

	}

	@Override
	void disable()
	{
		Util.consoleLog();

	}

	@Override
	void dispose()
	{
		Util.consoleLog();

		INSTANCE = null;
	}

	@Override
	protected void updateDS()
	{
		Util.consoleLog();

	}

}
