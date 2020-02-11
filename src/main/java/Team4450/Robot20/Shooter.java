package Team4450.Robot20;

import Team4450.Lib.Util;

public class Shooter extends SubSystem
{
	private Robot			robot;
	
	// This variable used to make this class is a singleton.
	
	private static Shooter 	INSTANCE = null;
	
	private Shooter (Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		Util.consoleLog("Shooter created!");
	}
		
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/	
	public static Shooter getInstance(Robot robot)
	{
		if (INSTANCE == null) INSTANCE = new Shooter(robot);
		
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
