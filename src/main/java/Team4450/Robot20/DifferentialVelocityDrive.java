package Team4450.Robot20;

import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Lib.Util;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtil;

/*
 * Notes: Completed 8-31-2020. Works but not really an improvement over
 * regular driving. However, due to Covid-19, we were never able to do
 * the characterization tests and get correct feed forward gains. The
 * FF gains used in this project are a guess and until we have the
 * correct gains, we can't really decide the value of velocity based
 * drive control. Its supposed to be smoother and a better way to 
 * control driving but as it stands now, it seems somewhat less smooth
 * and more touchy. Need to retest once we have correct gains and then
 * decide. 
 * 
 * One thing that is observed as that when you set power to zero, the
 * robot stops but then rocks back and forth one oscillation before 
 * motion stops. Suspect this is an aspect of the PID control used in
 * the voltage calculation.
 * 
 * Note we are driving based on a percent of max velocity but the 
 * characterization gains are in units of voltage so the result of
 * the pid and FF calculations is voltages representing velocity.
 * It all seems overly complicated but recommended by Wpilib developers.
 * Note that this code is based on Wpilib examples so it should do what
 * they say. Have to wait for characterization and correct gains to make
 * a final determination of the value of velocity driving. Note that the
 * FIRST examples of trapezoidal motion control and Ramsete path following
 * use this style of velocity based control through voltages.
 * 
 * Final note: during testing of this code it really became apparent that
 * running the drive talons in brake mode contributes to jerky robot motion.
 * This code turns off brakes as the driving default and motion seems better
 * in both normal and velocity driving. Note also that squared inputs and
 * setting the talon ramp rate to 0.5 second does not seem to make that
 * much difference.
 */

/**
 * Velocity drive controls drive base motors in tank configuration based on
 * target velocity, specified as -1 to +1 (%) input times the max velocity 
 * of the robot. Feed forward is computed from the desired velocity and the
 * ks and kv gain values determined by the Characterization process. These
 * values are in units of voltage so the feed forward calculation results
 * in a target voltage representing the path to target velocity. The PID 
 * controllers add an additional amount based on the error between target
 * velocity and actual velocity. Motors are controlled by setting voltage 
 * on the internally which compensates for battery
 * voltage decline as robot operates. Velocities are in meters/second.
 * You must measure max velocity, max angular velocity, ks and kv for each
 * robot that uses this class to get correct results. Use the FIRST robot
 * Characterization tool for this. Note this class must use the new Wpilib 
 * synchronous PID controller to work correctly. Do not change it to use
 * the threaded PID controller from either Wpilib or the copy in Robotlib.
 */
public class DifferentialVelocityDrive extends RobotDriveBase implements Sendable, AutoCloseable
{
	public final double maxSpeed; 			// meters per second.
	public final double maxAngularSpeed;	// radians per second.
	public final double trackWidth; 		// meters.
	public final double p, i, d, ks, kv;
	
	private final SpeedController				leftController, rightController;
	
	private final SRXMagneticEncoderRelative	leftEncoder, rightEncoder;

	private final PIDController 				leftPIDController;
	private final PIDController 				rightPIDController;
	
	private final DifferentialDriveKinematics 	kinematics;
	
	private final SimpleMotorFeedforward 		feedforward;
	
	private static int	instances;
	
	/**
	 * Constructs a Velocity drive object.
	 * @param leftController	Left side motor controller;
	 * @param rightController	Right side motor controller;
	 * @param leftEncoder		Left side encoder.
	 * @param rightEncoder		Right side encoder.
	 * @param trackWidth		Track width of robot in inches.
	 * @param maxSpeed			Max speed of robot in m/s.
	 * @param maxAngularSpeed	Max angular speed of robot in radians/s.
	 * @param p					P term for PID controllers. 12 / maxSpeed is a good default.
	 * @param i					I term for PID controllers. 0 is a good default.
	 * @param d					D term for PID controllers. 0 is a good default.
	 * @param ks				Feed forward static gain (volts). 1 is a good default.
	 * @param kv				Feed forward velocity gain (volts * seconds / distance). 1 is a good default.
	 */
	public DifferentialVelocityDrive(SpeedController leftController, 
						 			 SpeedController rightController,
						 			 SRXMagneticEncoderRelative leftEncoder, 
						 			 SRXMagneticEncoderRelative rightEncoder,
						 			 double trackWidth, double maxSpeed, double maxAngularSpeed,
						 			 double p, double i, double d, double ks, double kv) 
	{
		this.leftController = leftController;
		this.rightController = rightController;
		this.leftEncoder = leftEncoder;
		this.rightEncoder = rightEncoder;
		this.trackWidth = Util.inchesToMeters(trackWidth);
		this.maxSpeed = maxSpeed;
		this.maxAngularSpeed = maxAngularSpeed;
		this.p = p;
		this.i = i;
		this.d = d;
		this.ks = ks;
		this.kv = kv;
		
		leftPIDController = new PIDController(p, i, d);
		rightPIDController = new PIDController(p, i, d);
		
		kinematics = new DifferentialDriveKinematics(trackWidth);
		
		feedforward = new SimpleMotorFeedforward(ks, kv);
		
	    SendableRegistry.addChild(this, leftController);
	    SendableRegistry.addChild(this, rightController);
	    instances++;
	    SendableRegistry.addLW(this, getDescription(), instances);
	}
	
	/**
	 * Sets the desired wheel speeds by using current measured speeds and PID + Feed Forward
	 * controllers to calculate the motor voltage to sync actual to desired speed.
	 *
	 * @param speeds The desired wheel speeds in m/s.
	 */
	private void setSpeeds(DifferentialDriveWheelSpeeds speeds) 
	{
		// Feed forwards are in volts.
	    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
	    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
	    
	    final double leftSpeed = leftEncoder.getVelocity(PIDRateType.velocityMPS);
	    final double rightSpeed = rightEncoder.getVelocity(PIDRateType.velocityMPS);
	
	    // PID output is in m/s, as the error is target vel minus actual vel, multiplied by
	    // the PID factors. The P factor needs to scale the m/s error to voltage. Hence P for
	    // max vel of 2 m/s and 12 volts max power is 12/2 or 6. The error * 6 is target voltage.
	    
	    double leftOutput = leftPIDController.calculate(leftSpeed, speeds.leftMetersPerSecond);
	    double rightOutput = rightPIDController.calculate(rightSpeed, speeds.rightMetersPerSecond);
	    
	    // SetVoltage as a range of -12 to + 12. Internally, it is scaled by the actual battery voltage
	    // to arrive at a % output in the -1 to +1 range which is sent to controller set() method.
	    
	    leftController.setVoltage(leftOutput + leftFeedforward);
	    rightController.setVoltage(rightOutput + rightFeedforward);

//	    Util.consoleLog("lt=%.3f la=%.3f lo=%.3f lf=%.3f ls=%.3f - rt=%.3f ra=%.3f ro=%.3f rf=%.3f rs=%.3f",
//	    		speeds.leftMetersPerSecond, leftSpeed, leftOutput, leftFeedforward, leftController.get(),
//	    		speeds.rightMetersPerSecond, rightSpeed, rightOutput, rightFeedforward, rightController.get());
	    
	    feed();
	}
	
	/**
	 * Drives the robot with the given linear velocities for left and right motors. This is tank style.
	 * @param leftSpeed		Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 * @param rightSpeed	Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed)
	{
		// Dead band defaults to .02, max output defaults to 1.0.
		
	    leftSpeed = MathUtil.clamp(leftSpeed * m_maxOutput, -1.0, 1.0);
	    leftSpeed = applyDeadband(leftSpeed, m_deadband);

	    rightSpeed = MathUtil.clamp(rightSpeed * m_maxOutput, -1.0, 1.0);
	    rightSpeed = applyDeadband(rightSpeed, m_deadband);

	    setSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed * maxSpeed, rightSpeed * maxSpeed));
		
		//leftController.set(leftSpeed);
		//rightController.set(rightSpeed);
		
		feed();
	}
	
	/**
	 * Drives the robot with the given linear velocities for left and right motors. This is tank style.
	 * @param leftSpeed		Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 * @param rightSpeed	Linear velocity as a % of max speed. Ranges -1 to +1, + is forward unit is m/s.
	 * @param squaredInputs True to scale the input values with the Util.squareInput() function to make small
	 * 						inputs more gentle but still provide max speed.
	 */
	public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs)
	{
		// Dead band defaults to .02, max output defaults to 1.0.
		
	    leftSpeed = MathUtil.clamp(leftSpeed * m_maxOutput, -1.0, 1.0);
	    leftSpeed = applyDeadband(leftSpeed, m_deadband);

	    rightSpeed = MathUtil.clamp(rightSpeed * m_maxOutput, -1.0, 1.0);
	    rightSpeed = applyDeadband(rightSpeed, m_deadband);

	    if (squaredInputs)
			tankDrive(Util.squareInput(leftSpeed), Util.squareInput(rightSpeed));
		else
			tankDrive(leftSpeed, rightSpeed);
	}
	
	/**
	 * Drives the robot with the given percent voltage level without any
	 * PID control. This is tank style.
	 * @param leftPctPower	% of max voltage to apply. Ranges -1 to +1, + is forward.
	 * @param rightPctPower	% of max voltage to apply. Ranges -1 to +1, + is forward.
	 */
	public void tankDriveVolts(double leftPctPower, double rightPctPower)
	{
		// Dead band defaults to .02, max output defaults to 1.0.
		
	    leftPctPower = MathUtil.clamp(leftPctPower * m_maxOutput, -1.0, 1.0);
	    leftPctPower = applyDeadband(leftPctPower, m_deadband);

	    rightPctPower = MathUtil.clamp(rightPctPower * m_maxOutput, -1.0, 1.0);
	    rightPctPower = applyDeadband(rightPctPower, m_deadband);
	    
	    leftController.setVoltage(leftPctPower * 12.0);
	    rightController.setVoltage(rightPctPower * 12.0);
	    
	    feed();
	}
	
	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 * This is arcade style.
	 * @param speed 	Linear velocity as a % of max speed (forward/backward). Ranges -1 to +1, + is forward.
	 * @param rot		Angular velocity as a % of max angular speed. Ranges -1 to +1, + is right.
	 */
	public void arcadeDrive(double speed, double rotation)
	{
		// Dead band defaults to .02, max output defaults to 1.0.
		
	    speed = MathUtil.clamp(speed * m_maxOutput, -1.0, 1.0);
	    speed = applyDeadband(speed, m_deadband);

	    rotation = MathUtil.clamp(rotation * m_maxOutput, -1.0, 1.0);
	    rotation = applyDeadband(rotation, m_deadband);

	    // Note we invert rotation because we like to work + as clockwise, but ChassisSpeeds expects + to be counter
		// clockwise. Also note that the Y axis speed is zero since tank robot can't move sideways.
		
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(speed * maxSpeed, 
				0.0, -rotation * maxAngularSpeed));
		
	    setSpeeds(wheelSpeeds);
	}
	
	/**
	 * Drives the robot with the given linear velocity and angular velocity.
	 * This is arcade style.
	 * @param speed 	Linear velocity as a % of max speed (forward/backward). Ranges -1 to +1, + is forward.
	 * @param rot		Angular velocity as a % of max angular speed. Ranges -1 to +1, + is right.
	 * @param squaredInputs True to scale the input values with the Util.squareInput() function to make small
	 * 						inputs more gentle but still provide max speed.
	 */
	public void arcadeDrive(double speed, double rotation, boolean squaredInputs)
	{
		// Dead band defaults to .02, max output defaults to 1.0.
		
	    speed = MathUtil.clamp(speed * m_maxOutput, -1.0, 1.0);
	    speed = applyDeadband(speed, m_deadband);

	    rotation = MathUtil.clamp(rotation * m_maxOutput, -1.0, 1.0);
	    rotation = applyDeadband(rotation, m_deadband);

	    if (squaredInputs)
			arcadeDrive(Util.squareInput(speed), Util.squareInput(rotation));
		else
			arcadeDrive(speed, rotation);
	}
	
	/**
	 * Not Implemented.
	 * @param speed
	 * @param curve
	 * @param quickTurn
	 */
	public void curvatureDrive(double speed, double curve, boolean quickTurn)
	{
		
	}
	
	/**
	 * Stop motors.
	 */
	public void stopMotor()
	{
	    leftController.stopMotor();
	    rightController.stopMotor();
	    
	    feed();
	}

	/**
	 * Get object description.
	 * @return The object description.
	 */
	@Override
	public String getDescription()
	{
		return "DifferentialVelocityDrive";
	}

	/**
	 * Release any resources held by this object.
	 * @throws Exception
	 */
	@Override
	public void close() throws Exception
	{
	    SendableRegistry.remove(this);
	}

	/**
	 * Initialize this object as a Sendable. Not to be called by
	 * programmers.
	 * @param builder
	 */
	@Override
	public void initSendable(SendableBuilder builder) {
	    builder.setSmartDashboardType("DifferentialDrive");
	    builder.setActuator(true);
	    builder.setSafeState(this::stopMotor);
	    builder.addDoubleProperty("Left Motor Speed", leftController::get, leftController::set);
	    builder.addDoubleProperty("Right Motor Speed", rightController::get, rightController::set);
	}
}
