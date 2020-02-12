package Team4450.Robot20;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Channel extends SubSystem
{
	private Robot			robot;
	private boolean			beltRunning;
	
	// This variable used to make this class is a singleton.
	
	private static Channel 	INSTANCE = null;
	
	private Channel (Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		Util.consoleLog("Channel created!");
	}
		
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/	
	public static Channel getInstance(Robot robot)
	{
		if (INSTANCE == null) INSTANCE = new Channel(robot);
		
		return INSTANCE;		
	}

	@Override
	void enable()
	{
		Util.consoleLog();

		stop();
	}

	@Override
	void disable()
	{
		Util.consoleLog();

		stop();
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

		SmartDashboard.putBoolean("Belt", beltRunning);
	}

	public void stop()
	{
		Util.consoleLog();
		
		Devices.beltTalon.stopMotor();
		
		beltRunning = false;
		
		updateDS();
	}
	
	public void start(double power)
	{
		Util.consoleLog();
		
		Devices.beltTalon.set(power);
		
		beltRunning = true;
		
		updateDS();
	}
	
	public boolean isRunning()
	{
		return beltRunning;
	}
}

