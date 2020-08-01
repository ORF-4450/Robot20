package Team4450.Robot20;

import Team4450.Lib.NavX;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.SRXMagneticEncoderRelative.DistanceUnit;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;

/**
 * Odometer class for differential (tank) drive robot using
 * SRX Magnetic Encoders.
 * Tracks position of robot as it travels around the field.
 * Reliable for autonomous, not so much for teleop. User
 * must call update() on a regular basis for tracking to occur.
 */
public class DifferentialOdometer
{
	private SRXMagneticEncoderRelative	leftEncoder, rightEncoder;
	private NavX						navx;
	private DifferentialDriveOdometry	odometer;
	private double						cumulativeLeftCount, cumulativeRightCount;
	
	// This variable used to make this class is a singleton.
	
	private static DifferentialOdometer	INSTANCE = null;
	
	private DifferentialOdometer (SRXMagneticEncoderRelative leftEncoder, 
								  SRXMagneticEncoderRelative rightEncoder,
								  NavX navx)
	{
		Util.consoleLog();
		
		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder;
		this.navx = navx;
		
		odometer = new DifferentialDriveOdometry(navx.getTotalYaw2d());
		
		Util.consoleLog("DifferentialOdometry created!");
	}
		
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method. Creates initial instance if it does not already exist.
	* @param leftEncoder Left drive encoder.
	* @param rightEncoder Right drive encoder. 
	* @param navx NavX instance.
	* @return Reference to single shared instance of this class.
	*/	
	public static DifferentialOdometer getInstance(SRXMagneticEncoderRelative leftEncoder, 
			  									   SRXMagneticEncoderRelative rightEncoder,
			  									   NavX navx)
	{
		if (INSTANCE == null) INSTANCE = new DifferentialOdometer(leftEncoder, rightEncoder, navx);
		
		return INSTANCE;		
	}
	
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method. Assumes initial instance already exists.
	* @return Reference to single shared instance of this class.
	* @throws Exception if instance not already initialized.
	*/	
	public static DifferentialOdometer getInstance() 
	{
		if (INSTANCE == null) throw new Error("DifferentialOdometer not initialized");
		
		return INSTANCE;		
	}
	
	/**
	 * Update the odometer with current encoder distances and NavX angle.
	 * @return Updated pose (distances in meters).
	 */
	public Pose2d update()
	{
		// Odometer uses total encoder count since start of match (or odometer reset).
		// So we need to track that as encoders can be reset at any time to facilitate
		// navigation functions along the way.
		
		cumulativeLeftCount += leftEncoder.getDistance(DistanceUnit.Meters);
		cumulativeRightCount += leftEncoder.getDistance(DistanceUnit.Meters);
		
		return odometer.update(navx.getTotalYaw2d(), cumulativeLeftCount, cumulativeRightCount);
	}
	
	/**
	 * Get current pose from odometer. Pose distances in meters.
	 * Pose X is distance along the long side of the field from your driver
	 * station wall. Y is distance along the short side of the field starting
	 * on the left. Angle is referenced from zero as pointing directly at the
	 * opposition driver station wall, + is left, - is right of that zero
	 * alignment.
	 * @return Current pose.
	 */
	public Pose2d getPose()
	{
		return odometer.getPoseMeters();
	}
	
	/**
	 * Reset odometer to new pose and angle. Resets encoders. You must reset
	 * the Navx angle manually.
	 * @param pose New starting pose.
	 * @param angle Current gyro angle used to offset future angle measurements
	 * acting to reset odometer angle without resetting gyro.
	 */
	public void reset(Pose2d pose, Rotation2d angle)
	{
		odometer.resetPosition(pose, angle);
		
		cumulativeLeftCount = 0;
		cumulativeRightCount = 0;
	}
}
