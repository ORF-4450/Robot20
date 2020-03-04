package Team4450.Robot20;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubSystem
{
	private Robot			robot;
	private boolean			brakeEngaged;
	
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

		Devices.winchEncoder.reset();
		
		releaseBrake();
	}

	@Override
	void disable()
	{
		Util.consoleLog();
		
		// Note: we don't release brake on disable because we want the robot to hang
		// on the brake when match is over.

	}

	@Override
	void dispose()
	{
		Util.consoleLog();
		
		disable();
		
		INSTANCE = null;
	}
	
	/**
	 * Set power level for climber winch motors.
	 * @param power -1 to +1, + is up because we pull the stick back to climb.
	 */
	public void set(double power)
	{
		// If trying to go down (-) and switch returns true, we are at bottom so kill the power.
		
		if (power < 0 && Devices.winchSwitch.get()) 
		{
			Devices.winchEncoder.reset();
			power = 0;
		}
		
		// If trying to go up (+) and encoder is at upper limit count, we are the top kill the power.
		if (power > 0 && Devices.winchEncoder.get() >= 5300) power = 0;

		Devices.winchDrive.set(power);
	}
	
	public void engageBrake()
	{
		Util.consoleLog();
		
		Devices.climberBrake.SetA();
		
		brakeEngaged = true;
		
		updateDS();
	}
	
	public void releaseBrake()
	{
		Util.consoleLog();
		
		Devices.climberBrake.SetB();
		
		brakeEngaged = false;
		
		updateDS();
	}
	
	public boolean isBrakeEngaged()
	{
		return brakeEngaged;
	}

	@Override
	protected void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Brake", brakeEngaged);
	}
}
