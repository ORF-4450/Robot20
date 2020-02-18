package Team4450.Robot20;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import Team4450.Lib.JoyStick;
import Team4450.Lib.LaunchPad;
import Team4450.Lib.NavX;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import Team4450.Lib.JoyStick.JoyStickButtonIDs;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("removal")
public class Devices
{
	  // Motor CAN ID/PWM port assignments (1=left-front, 2=left-rear, 3=right-front, 4=right-rear)
	  public static WPI_TalonSRX		LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon, shooterTalon;
	  
	  public static WPI_VictorSPX		pickupVictor, beltVictor, winchFrontVictor, winchBackVictor;
	  public static WPI_VictorSPX		hookVictor, colorWheelVictor;
	  
	  public static DifferentialDrive	robotDrive;
	  
	  public static SpeedControllerGroup	winchDrive;
	  
	  public static JoyStick			rightStick = null;
	  public static JoyStick			leftStick = null;
	  public static JoyStick			utilityStick = null;
	  public static LaunchPad			launchPad = null;

	  public final static Compressor	compressor = new Compressor(0);		// Compressor class represents the PCM.

	  public final static ValveDA		highLowValve = new ValveDA(0);			// For gearbox.
	  public final static ValveDA		pickupValve = new ValveDA(2);			// For pickup arm.
	  
	  public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	  public final static PowerDistributionPanel	pdp = new PowerDistributionPanel();

	  public final static DriverStation				ds = DriverStation.getInstance();

	  public static NavX				navx;
	  
	  public static VisionLL			visionLL;
		
	  //public static AnalogGyro			gyro = new AnalogGyro(1);

	  // Encoder (regular type) is plugged into dio port n:
	  // orange=+5v blue=signal, dio port n+1: black=gnd yellow=signal. 
	  //public final static Encoder		winchEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	  
	  // SRX magnetic encoder plugged into a CAN Talon.
	  public static SRXMagneticEncoderRelative	leftEncoder, rightEncoder, shooterEncoder;
	  
	  private static boolean			talonBrakeMode;
	  
	  public static GearBox				gearBox;
	  public static Shooter				shooter;
	  public static Climber				climber;
	  public static Pickup				pickup;
	  public static Channel				channel;
	  
	  // Private constructor prevents creation of any instances of this "static" class.
	  
	  private Devices() {}
	  
	  // Initialize motor controllers, groups and encoders.
	  
	  public static void configureDevices(Robot robot)
	  {
		  Util.consoleLog();

		  // Create the drive Talons.
		  LFCanTalon = new WPI_TalonSRX(1);
		  LRCanTalon = new WPI_TalonSRX(2);
		  RFCanTalon = new WPI_TalonSRX(3);	
		  RRCanTalon = new WPI_TalonSRX(4);	
		  
		  //shooterTalon = new WPI_TalonSRX(5);
		  pickupVictor = new WPI_VictorSPX(6);
		  beltVictor = new WPI_VictorSPX(7);
		  winchFrontVictor = new WPI_VictorSPX(8);
		  winchBackVictor = new WPI_VictorSPX(9);
		  hookVictor = new WPI_VictorSPX(10);
		  colorWheelVictor = new WPI_VictorSPX(11);
		  
	      // Initialize CAN Talons and write status to log so we can verify
	      // all the Talons are connected.
	      InitializeCANTalon(LFCanTalon);
	      InitializeCANTalon(LRCanTalon);
	      InitializeCANTalon(RFCanTalon);
	      InitializeCANTalon(RRCanTalon);
	      
//	      InitializeCANTalon(shooterTalon);
//	      InitializeCANTalon(beltTalon);
//	      InitializeCANTalon(winchFrontTalon);
//	      InitializeCANTalon(winchBackTalon);
//	      InitializeCANTalon(hookTalon);

	      // Configure CAN Talons with correct inversions.
	      LFCanTalon.setInverted(true);
		  LRCanTalon.setInverted(true);
		  
		  RFCanTalon.setInverted(true);
		  RRCanTalon.setInverted(true);

	      // Turn on brake mode for drive CAN Talons.
	      SetCANTalonBrakeMode(true);
	      
//	      shooterTalon.setInverted(true);
//	      shooterTalon.setNeutralMode(NeutralMode.Coast);
	      
	      beltVictor.setNeutralMode(NeutralMode.Brake);
	      winchFrontVictor.setNeutralMode(NeutralMode.Brake);
	      winchBackVictor.setNeutralMode(NeutralMode.Brake);
	      hookVictor.setNeutralMode(NeutralMode.Brake);
	      colorWheelVictor.setNeutralMode(NeutralMode.Brake);
	      
	      // For 2020 robot, put rear talons into a differential drive object and set the
	      // front talons to follow the rears.
		  
		  LFCanTalon.set(ControlMode.Follower, LRCanTalon.getDeviceID());
		  RFCanTalon.set(ControlMode.Follower, RRCanTalon.getDeviceID());
		  
		  robotDrive = new DifferentialDrive(LRCanTalon, RRCanTalon);
		  
		  // Configure SRX encoders as needed for measuring velocity and distance. 
		  // 5.8 is wheel diameter in inches. Adjust for each years robot.
		  rightEncoder = new SRXMagneticEncoderRelative(RRCanTalon, 7.5);
		  leftEncoder = new SRXMagneticEncoderRelative(LRCanTalon, 7.5);
		  
		  leftEncoder.setInverted(true);
		  
		  //shooterEncoder = new SRXMagneticEncoderRelative(shooterTalon, 5.8);
		  
		  winchDrive = new SpeedControllerGroup(winchFrontVictor, winchBackVictor);

   		  // Create launch pad with all buttons monitored and auto start of monitoring loop.
   		  // Will add event handler in Teleop class.
 		  launchPad = new LaunchPad(new Joystick(3));

 		  // Create our JoyStick classes for each JS with selective button monitoring. Must start monitoring
 		  // loop manually when not adding all buttons. Will add event handler in Teleop class.
 		  leftStick = new JoyStick(new Joystick(0), "LeftStick", JoyStickButtonIDs.TRIGGER);
 		  leftStick.Start();

 		  rightStick = new JoyStick(new Joystick(1), "RightStick", JoyStickButtonIDs.TRIGGER);
 		  //Example on how to track an additional button:
 		  rightStick.AddButton(JoyStickButtonIDs.TOP_BACK);
 		  rightStick.Start();

 		  // Create utility stick with all buttons monitored and auto start.
 		  utilityStick = new JoyStick(new Joystick(2), "UtilityStick");
 		  
 		  // Create instances of the singleton subsystem classes.
 		  
 		  gearBox = GearBox.getInstance(robot);
 		  
 		  shooter = Shooter.getInstance(robot);
 		  climber = Climber.getInstance(robot);
 		  pickup = Pickup.getInstance(robot);
 		  channel = Channel.getInstance(robot);
 		  
 		  visionLL = VisionLL.getInstance(robot);
	  }

	  // Initialize and Log status indication from CANTalon. If we see an exception
	  // or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	  public static void InitializeCANTalon(WPI_TalonSRX talon)
	  {
		  Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		  talon.clearStickyFaults(0); //0ms means no blocking.
	  }
	  
	  // Set neutral behavior of drive CAN Talons. True = brake mode, false = coast mode.

	  public static void SetCANTalonBrakeMode(boolean brakeMode)
	  {
		  Util.consoleLog("brakes on=%b", brakeMode);
		  
		  SmartDashboard.putBoolean("Brakes", brakeMode);

		  talonBrakeMode = brakeMode;
		  
		  NeutralMode newMode;
		  
		  if (brakeMode) 
			  newMode = NeutralMode.Brake;
		  else 
			  newMode = NeutralMode.Coast;
		  
		  LFCanTalon.setNeutralMode(newMode);
		  LRCanTalon.setNeutralMode(newMode);
		  RFCanTalon.setNeutralMode(newMode);
		  RRCanTalon.setNeutralMode(newMode);
	  }
	  
	  public static boolean isBrakeMode()
	  {
		  return talonBrakeMode;
	  }
	  
	  // Set CAN Talon voltage ramp rate. Rate is number of seconds from zero to full output.
	  // zero disables.
	  
	  public static void SetCANTalonRampRate(double seconds)
	  {
		  Util.consoleLog("%.2f", seconds);
		  
		  LFCanTalon.configOpenloopRamp(seconds, 0);
		  LRCanTalon.configOpenloopRamp(seconds, 0);
		  RFCanTalon.configOpenloopRamp(seconds, 0);
		  RRCanTalon.configOpenloopRamp(seconds, 0);
	  }
	  
	  // Return voltage and current draw for each CAN Talon.
	  
	  public static String GetCANTalonStatus()
	  {
		  return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  LFCanTalon.getMotorOutputVoltage(), LFCanTalon.getOutputCurrent(),
				  LRCanTalon.getMotorOutputVoltage(), LRCanTalon.getOutputCurrent(),
				  RFCanTalon.getMotorOutputVoltage(), RFCanTalon.getOutputCurrent(),
				  RRCanTalon.getMotorOutputVoltage(), RRCanTalon.getOutputCurrent()
				  );
	  }
}
