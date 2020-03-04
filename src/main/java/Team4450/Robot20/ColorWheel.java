package Team4450.Robot20;

import com.revrobotics.ColorMatchResult;

import Team4450.Lib.RevColorSensor;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class ColorWheel extends SubSystem
{
	private Robot			robot;
	private boolean			wheelRunning, countingTurns, rotatingToTarget;
	private Thread			countTurnsThread, rotateToTargetThread;
	
  	RevColorSensor			colorSensor = RevColorSensor.getInstance();

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

		stopWheel();
	}

	@Override
	void disable()
	{
		Util.consoleLog();

		stopWheel();
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
		SmartDashboard.putBoolean("CountingTurns", countingTurns);
		SmartDashboard.putBoolean("RotatingToTarget", rotatingToTarget);
	}

	public void stopWheel()
	{
		Util.consoleLog();
		
		if (countingTurns) stopCountingTurns();
		
		if (rotatingToTarget) stopRotateToTarget();

		Devices.colorWheelVictor.stopMotor();
		
		wheelRunning = false;
		
		updateDS();
	}
	
	public void startWheel(double power, boolean gameColor)
	{
		Util.consoleLog();
		
		//if (gameColor)
		//	startRotateToTarget();
		//else
		//startCountingTurns();
		
		Devices.colorWheelVictor.set(power);
		
		wheelRunning = true;
		
		updateDS();
	}
	
	public boolean isRunning()
	{
		return wheelRunning;
	}
	
	public void setTargetToCurrentColor()
	{
		Color targetColor = colorSensor.getColor();
		
		Util.consoleLog(targetColor.toString());
		
		colorSensor.resetColorMatcher();
		
		if (targetColor != null) colorSensor.addColorMatch(targetColor);
	}
	
	// Reads game data and if color code present, loads the correct Color
	// object into the Color Sensor matching function.
	
	public void setGameTargetColor()
	{
		Color	targetColor = null;
		String 	gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		gameData = "R";
		
		Util.consoleLog(gameData);
		
		colorSensor.resetColorMatcher();

		if(gameData.length() > 0)
		{
			switch (gameData.charAt(0))
			{
				case 'B' :
					//Blue code.
					targetColor = RevColorSensor.getMatchColor(0.143, 0.427, 0.429);
					break;
					
				case 'G' :
					//Green code.
					targetColor = RevColorSensor.getMatchColor(0.197, 0.561, 0.240);
					break;
					
				case 'R' :
					//Red code.
					targetColor = RevColorSensor.getMatchColor(0.561, 0.232, 0.114);
					break;
					
				case 'Y' :
					//Yellow code.
					targetColor = RevColorSensor.getMatchColor(0.361, 0.524, 0.113);
					break;
					
				default :
					//This is corrupt data.
					break;
		  }
		} else {
			// no data received yet default to red.
			//targetColor = RevColorSensor.getMatchColor(0.561, 0.232, 0.114);
		}		
		
		if (targetColor != null) colorSensor.addColorMatch(targetColor);
	}
	
	// Match current color read from sensor to the target color in the color matcher
	// and returns true if current color matches target color with confidence
	// >= 85%.
	
	public boolean colorMatch()
	{
		Color color = colorSensor.getColor();

		ColorMatchResult matchResult = colorSensor.matchClosestColor(color);

		//LCD.printLine(6, "color match result r=%f g=%f b=%f  conf=%f", matchResult.color.red, matchResult.color.green,
		//		matchResult.color.blue, matchResult.confidence);

		if (matchResult.confidence >= .85)
			return true;
		else
			return false;
	}
	
	private void startCountingTurns()
	{
		Util.consoleLog();
		
		if (countTurnsThread != null) return;

		countTurnsThread = new CountTurns();
		
		countTurnsThread.start();
	}
	
	private void stopCountingTurns()
	{
		Util.consoleLog();

		if (countTurnsThread != null) countTurnsThread.interrupt();
		
		countTurnsThread = null;
	}
	
	private class CountTurns extends Thread
	{
		int		turnCount;
		
		CountTurns()
		{
			Util.consoleLog();
			
			this.setName("CountTurns");
	    }
		
	    public void run()
	    {
	    	boolean onTargetColor;
	    	
	    	Util.consoleLog();	

	    	countingTurns = true;
			
	    	// Current color is color we will count when it passes by the sensor
	    	// after the first time. We have to detect color change and count the
	    	// change if it changes back to target color from not target color.
	    	
			setTargetToCurrentColor();
			
			onTargetColor = true;

	    	updateDS();
	    	
	    	try
	    	{
    	    	while (!isInterrupted() && robot.isEnabled())
    	    	{
    	    		if (colorMatch())
    	    		{
    	    			if (!onTargetColor)
    	    			{
    	    				onTargetColor = true;
    	    				turnCount++;
    	    			}
    	    		} else onTargetColor = false;
    	    		
    	    		if (turnCount > 3) 
    	    		{
    	    			Devices.colorWheelVictor.stopMotor();
    	    			wheelRunning = false;
    	    			break;
    	    		}
    	    		
    	    		sleep(250);
    	    	}
	    	}
	    	catch (InterruptedException e) { }
	    	catch (Throwable e) { e.printStackTrace(Util.logPrintStream); }

	    	countingTurns = false;
	    	countTurnsThread = null;
	    	
	    	updateDS();
	    }
	}
	
	private void startRotateToTarget()
	{
		Util.consoleLog();
		
		if (countTurnsThread != null) return;

		countTurnsThread = new CountTurns();
		
		countTurnsThread.start();
	}
	
	private void stopRotateToTarget()
	{
		Util.consoleLog();

		if (countTurnsThread != null) countTurnsThread.interrupt();
		
		countTurnsThread = null;
	}
	
	private class RotateToTarget extends Thread
	{
		RotateToTarget()
		{
			Util.consoleLog();
			
			this.setName("RotateToTarget");
	    }
		
	    public void run()
	    {
	    	Util.consoleLog();	

	    	rotatingToTarget = true;

	    	setGameTargetColor();

	    	updateDS();
	    	
	    	try
	    	{
    	    	while (!isInterrupted() && robot.isEnabled())
    	    	{
    	    		if (colorMatch())
    	    		{
    	    			Devices.colorWheelVictor.stopMotor();
    	    			wheelRunning = false;
    	    			break;
    	    		}
    	    		
    	    		sleep(250);
    	    	}
	    	}
	    	catch (InterruptedException e) { }
	    	catch (Throwable e) { e.printStackTrace(Util.logPrintStream); }

	    	rotatingToTarget = false;
	    	rotateToTargetThread = null;
	    	
	    	updateDS();
	    }
	}
}
