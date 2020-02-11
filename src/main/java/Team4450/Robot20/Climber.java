package Team4450.Robot20;

import Team4450.Lib.Util;

public class Climber extends SubSystem
{
	private Robot			robot;
	
	// This variable used to make this class is a singleton.
	
	private static Climber 	INSTANCE = null;
	
	private Climber (Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		Util.consoleLog("Climber created!");
	}
		
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/	
	public static Climber getInstance(Robot robot)
	{
		if (INSTANCE == null) INSTANCE = new Climber(robot);
		
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
		
		disable();
		
		INSTANCE = null;
	}

	@Override
	protected void updateDS()
	{
		Util.consoleLog();

	}

}
