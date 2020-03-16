package Team4450.Robot20;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;
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
		
		// Configure interrupt handler for the ballEye optical ball detector.
		
		Devices.ballEye.requestInterrupts(new InterruptHandler());
		
		// Listen for a falling edge interrupt.
		
		Devices.ballEye.setUpSourceEdge(false, true);

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
		SmartDashboard.putBoolean("PickupExtended", extended);
	}
	
	public void extend()
	{
		Util.consoleLog();
		
		Devices.pickupValve.SetA();
		
		extended = true;
		
		start(.50);
		
		updateDS();
	}
	
	public void retract()
	{
		Util.consoleLog();
		
		stop();

		Devices.pickupValve.SetB();
		
		extended = false;
		
		updateDS();
	}
	
	public void start(double power)
	{
		Util.consoleLog("%.2f", power);
		
		Devices.pickupTalon.set(power);
		
		pickupRunning = true;
		
		Devices.ballEye.enableInterrupts();
		
		updateDS();
	}

	public void stop()
	{
		Util.consoleLog();
		
		Devices.pickupTalon.stopMotor();
	
		pickupRunning = false;
		
		Devices.ballEye.disableInterrupts();
	
		updateDS();
	}
	
	public boolean isExtended()
	{
		return extended;
	}
	
	public boolean isRunning()
	{
		return pickupRunning;
	}
		
	// Interrupt handler object to handle detection of ball by the ball Eye
	// optical sensor. The sensor generates a hardware interrupt when the 
	// eye is triggered and the fired method is called when interrupt occurs.
	// Keep the length of the code in that method short as no new interrupts
	// will be reported until fired method ends.
	private class InterruptHandler extends InterruptHandlerFunction<Object> 
	{
	     @Override
	     public void interruptFired(int interruptAssertedMask, Object param) 
	     {
	    	 Util.consoleLog("ball  interrupt");
	    	 
	    	 Devices.channel.intakeBall();
	    	 
	    	 Channel channel = (Channel) param;
	    	 channel.intakeBall();
	     }
	     
	     @SuppressWarnings("unused")
		 public Channel overridableParamter()
	     {
			return Devices.channel;
	     }
	}
}
