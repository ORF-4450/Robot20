package Team4450.Robot20;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pickup extends SubSystem
{
	private Robot		robot;
	private boolean		extended = false, pickupRunning = false, ballDetection = false;
	private Thread		ballDetectorThread;

	// This variable used to make this class is a singleton.
	
	private static Pickup 	INSTANCE = null;
	
	private Pickup (Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		// Configure interrupt handler for the ballEye optical ball detector.
		
		Devices.ballEye.requestInterrupts(new InterruptHandlerFunction<Object>() 
		{

		     @Override
		     public void interruptFired(int interruptAssertedMask, Object param) 
		     {
		    	 Util.consoleLog("ball  interrupt");
		     }
		});
		
		// Listen for a falling edge
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
		
		startBallDetector();
		
		updateDS();
	}

	public void stop()
	{
		Util.consoleLog();
		
		Devices.pickupTalon.stopMotor();
	
		pickupRunning = false;
		
		stopBallDetector();
	
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
	
	public void startBallDetector()
	{
		Util.consoleLog();
		
		if (ballDetectorThread != null) return;

		ballDetectorThread = new BallDetector();
		
		ballDetectorThread.start();
	}
	
	private void stopBallDetector()
	{
		Util.consoleLog();

		if (ballDetectorThread != null) ballDetectorThread.interrupt();
		
		ballDetectorThread = null;
	}
	
	private class BallDetector extends Thread
	{
		BallDetector()
		{
			Util.consoleLog();
			
			this.setName("BallDetector");
	    }
		
	    public void run()
	    {
	    	Util.consoleLog();	

	    	ballDetection = true;

	    	updateDS();
	    	
	    	try
	    	{
    	    	while (!isInterrupted() && robot.isEnabled())
    	    	{
    	    		if (!Devices.ballEye.get()) Util.consoleLog("ball detected");
    	    		sleep(10);
    	    	}
	    	}
	    	catch (InterruptedException e) { }
	    	catch (Throwable e) { e.printStackTrace(Util.logPrintStream); }

	    	ballDetection = false;
	    	ballDetectorThread = null;
	    	
	    	updateDS();
	    }
	}
	
	public void enableBalldetector()
	{
		Util.consoleLog();
		
		Devices.ballEye.requestInterrupts(new InterruptHandlerFunction<Object>() 
		{

		     @Override
		     public void interruptFired(int interruptAssertedMask, Object param) 
		     {
		    	 Util.consoleLog("ball  interrupt");
		     }
		});
		
		// Listen for a falling edge
		Devices.ballEye.setUpSourceEdge(false, true);
		// Enable digital interrupt pin
		Devices.ballEye.enableInterrupts();
	}
}
