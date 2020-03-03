package Team4450.Robot20;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorWheel extends SubSystem
{
	private Robot			robot;
	private boolean			wheelRunning;
	
	// This variable used to make this class is a singleton.
	
	private static ColorWheel	INSTANCE = null;
	
	private ColorWheel (Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		Util.consoleLog("ColorWheel created!");
	}
		
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/	
	public static ColorWheel getInstance(Robot robot)
	{
		if (INSTANCE == null) INSTANCE = new ColorWheel(robot);
		
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

		SmartDashboard.putBoolean("Wheel", wheelRunning);
	}

	public void stop()
	{
		Util.consoleLog();
		
		Devices.colorWheelVictor.stopMotor();
		
		wheelRunning = false;
		
		updateDS();
	}
	
	public void start(double power)
	{
		Util.consoleLog();
		
		Devices.colorWheelVictor.set(power);
		
		wheelRunning = true;
		
		updateDS();
	}
	
	public boolean isRunning()
	{
		return wheelRunning;
	}
}
