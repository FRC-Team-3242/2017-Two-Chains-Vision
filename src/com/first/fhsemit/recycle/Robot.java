
package com.first.fhsemit.recycle;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	
	I2C lights;
	
	Preferences pref;
	SendableChooser autonomousChooser;
	SendableChooser turnChooser;
	int autonomousMode;
	
	Joystick controller;
	RobotDrive drive;
	Hooks hooks;
	Tilter tilter;
	//DigitalInput ramButton;
	//RangeFinder rangeFinder;
	int conveyorState;
	int autonomousState;
	int turnState;
	TeleopDrive teleopDrive;
	Timer autonomousTimer;
	
    public void robotInit() {
    	controller = new Joystick(1);
    	
    	//negative is forward, and positive is backwards for Y
    	//positive is clockwise and negative is counterclockwise for turning
    	//drive motors are backwards
    	CANTalon frontLeft = new CANTalon(4);
    	CANTalon backLeft = new CANTalon(3);
    	CANTalon backRight = new CANTalon(2);
    	CANTalon frontRight = new CANTalon(1);
    	drive = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
    	drive.setInvertedMotor(MotorType.kFrontLeft, true);
    	drive.setInvertedMotor(MotorType.kRearLeft, true);
    	//frontLeft.setP(1.2);
    	teleopDrive = new TeleopDrive(drive);
    	
    	hooks = new Hooks(5,6,new DigitalInput(0),new DigitalInput(1),new DigitalInput(2));
    	tilter = new Tilter(new Relay(2),new AnalogInput(1));
    	
    	
    	//rangeFinder = new RangeFinder(new AnalogInput(1));
    	
    	autonomousTimer = new Timer();
    	autonomousTimer.start();
    	
    	lights = new I2C(I2C.Port.kMXP,0); //TODO uncomment
    	//lights.write(1, 0xf);
    	
    	pref = Preferences.getInstance();
    	autonomousChooser = new SendableChooser();
    	autonomousChooser.addDefault("Strafe", 1);
    	autonomousChooser.addObject("Only Trashcan", 2);
    	autonomousChooser.addObject("Forward then turn", 0);
    	autonomousChooser.addObject("Only Back Up", 3);
    	turnChooser = new SendableChooser();
    	turnChooser.addDefault("Turn Right", 2);
    	turnChooser.addObject("No Turn", 0);
    	turnChooser.addObject("Turn left", 1);
    	SmartDashboard.putData("Autonomous Mode",autonomousChooser);
    	SmartDashboard.putData("Turn Mode",turnChooser);
    	loadPrefs();
    }
    
    
    private void loadPrefs(){
    	hooks.expo = pref.getBoolean("expo",true);
    	hooks.sensitivity = pref.getDouble("HookSpeed",0.6);
    	
    	teleopDrive.strafeSpeed = pref.getDouble("StrafeSpeed",0.26);
    	teleopDrive.slowStrafeSpeed = pref.getDouble("SlowStrafeSpeed",0.125);
    	teleopDrive.turnSpeed = pref.getDouble("TurnSpeed",-0.8);//0.25;
    	teleopDrive.forwardSpeed = pref.getDouble("ForwardSpeed",-1);//0.4;
    	pref.save();
    }

    
    @Override
    public void teleopInit(){
    	hooks.state = 0;
    	conveyorState = 0;
    	hooks.numberToMove = 0;
    	tilter.centeringState = 0;
    	hooks.brake(true);
    	loadPrefs();
    	hooks.expo = true;
    }

    @Override
    public void teleopPeriodic() {
    	//SmartDashboard.putNumber("Range Finder", rangeFinder.finder.getVoltage());
    	//manualConveyor();
    	//autoConveyor();
    	incrementalConveyor();
    	teleopDrive.drive(controller);
        //teleopDrive();
    }
    
    
    private void manualConveyor(){
    	//manual conveyor control
    	hooks.move(controller.getRawAxis(5) + 0.06);
    	
    	//manual tilter control
    	tilter.move(controller);
    }
    
    private void incrementalConveyor(){
    	//SmartDashboard.putNumber("hooks number to move: ", hooks.numberToMove); //TODO remove
    	
    	switch(conveyorState){
    	case 0:
    		if(controller.getRawButton(6)){
    			hooks.numberToMove++;
    			conveyorState++;
    		}else if(controller.getRawButton(5)){
        		hooks.numberToMove--;
        		conveyorState++;
        	}
    		break;
    	case 1:
    		if(!controller.getRawButton(6) && !controller.getRawButton(5)) conveyorState = 0;
    		break;
    	}
    	if(controller.getRawButton(10)) hooks.numberToMove = 0;//R3 -> emergency stop
    	//if(controller.getRawButton(6)) hooks.numberToMove++;
    	//if(controller.getRawButton(5)) hooks.numberToMove--;
    	if(hooks.autoMove()) manualConveyor();
    	
    }
    
    
    

    public void autonomousInit(){
    	autonomousMode = (int) autonomousChooser.getSelected();
    	turnState = (int) turnChooser.getSelected();
    	if(autonomousMode == 3){
    		autonomousState = 16;
    	}else
    		autonomousState = 1;
    	
    	autonomousTimer.reset();
    	hooks.expo = false;
    }
    
    
    //19
    public void oldAutonomousPeriodic() {
    	//SmartDashboard.putNumber("AutoState", autonomousState);
    	switch(autonomousState){
		/*case 0://move to first object
			if(!autonomousTimer.hasPeriodPassed(0.1)){
				drive.mecanumDrive_Cartesian(0, 0.25F, 0, 0);//TODO cfg
			}else{
				autonomousState++;
				drive.stopMotor();
			}
			break;*/
    	case 0://plumb the tilter
    		if(tilter.straighten()){
    			autonomousState = 1;
    		}
    		break;
		case 1://pick up first object
			hooks.brake(false);
			if(hooks.moveOne(hooks::goUp,0.45)){//TODO .209 .6
				if(autonomousMode == 0){
					autonomousState = 11;
					hooks.brake(true);
				}else if(autonomousMode == 1 || autonomousMode == 2 || autonomousMode == 3){
					autonomousState = 17;
				}
				autonomousTimer.reset();
			}
			break;
		case 17://make room for tote
			if(hooks.moveOne(hooks::goUp,0.45)){//TODO .209 .55
				hooks.brake(true);
				if(autonomousMode == 3){
					autonomousState = 16;
				}else if(autonomousMode < 3){
					autonomousState = 15;
				}
				autonomousTimer.reset();
			}
			break;
		case 11://tilt back
			if(!autonomousTimer.hasPeriodPassed(0.8)){
				tilter.reverse();
			}else{
				if(autonomousMode == 0){
					autonomousState = 2;
				}else if(autonomousMode == 1 || autonomousMode == 2){
					autonomousState = 15;
				}
				tilter.off();
			}
			break;
		case 2://move to second object
			//if(!rangeFinder.isInRange()){
			if(!autonomousTimer.hasPeriodPassed(2)){//1.4
				drive.mecanumDrive_Cartesian(0, 0.16F, 0, 0);//0.185 - 0.17
			}else{
				autonomousState=10;
				drive.stopMotor();
			}
			break;
		case 15://Strafe to second object
			if(!autonomousTimer.hasPeriodPassed(1.63)){//1.4
				drive.mecanumDrive_Cartesian(0.5, 0.02, 0.02, 0);
			}else{

				if(autonomousMode == 0)
					autonomousState = 10;
				else if(autonomousMode == 1 || autonomousMode == 2){
					autonomousState = 13;
				}
				drive.stopMotor();
			}
			break;
		case 10://tilt forward
			if(!autonomousTimer.hasPeriodPassed(0.8)){
				tilter.forward();
			}else{
				autonomousState = 13;
				tilter.off();
			}
			break;
		case 13://move down
			if(!autonomousTimer.hasPeriodPassed(0.18)){//1.4
				hooks.goDown(0.2);
			}else{
				
				autonomousState=14;
				hooks.stop();
			}
			break;
		case 14://move forward to second object
			if(!autonomousTimer.hasPeriodPassed(1.4)){//1.4
				drive.mecanumDrive_Cartesian(0, 0.19F, 0, 0);//0.16
			}else{
				autonomousState=12;
				drive.stopMotor();
			}
			break;
		case 12://picking up object
			hooks.brake(false);
			if(hooks.moveOne(hooks::goUp,0.4)){//0.35 - 0.31 0.6 0.5
				//autonomousState++;
				autonomousState = 3;
			}
			break;
		case 3://pick up second object
			if(hooks.moveOne(hooks::goUp,0.4)){//0.35 - 0.31 0.6
				//autonomousState++;
				hooks.brake(true);
				if(this.autonomousMode == 0){
					autonomousState = 4;
				}else if(autonomousMode == 1){
					autonomousState = 16;
				}else if(autonomousMode == 2)
					autonomousState = 18;
				autonomousTimer.reset();
			}
			break;
		case 4://turn left towards autozone
			if(!autonomousTimer.hasPeriodPassed(1.57)){
				drive.mecanumDrive_Cartesian(0, 0, 0.37 - autonomousTimer.get()*0.05, 0);//TODO cfg
				//if(autonomousTimer.get()<0.3){
				//	tilter.reverse();
				//}else{
				//	tilter.off();
				//}
			}else{
				drive.mecanumDrive_Cartesian(0, 0, -0.003, 0);//stop
				if(autonomousMode == 1 || autonomousMode == 2 || autonomousMode == 3){
					autonomousState = 6;
				}else if(autonomousMode == 0){
					autonomousState=5;
				}
			}
			break;
		case 5://move to autoZone
			if(!autonomousTimer.hasPeriodPassed(1.4)){//TODO cfg
				drive.mecanumDrive_Cartesian(0, 0.575, 0, 0);//TODO cfg
			}else{
				autonomousState=6;
				drive.stopMotor();
			}
			break;
		case 16://back up to autozone
			if(!autonomousTimer.hasPeriodPassed(1.25)){//TODO 1.4
				drive.mecanumDrive_Cartesian(0, -0.575, 0, 0);//TODO cfg
			}else{
				if(turnState == 0){
					autonomousState=6;
				}else if(turnState == 1){
					autonomousState = 4;
				}else if(turnState == 2){
					autonomousState = 19;
				}
				drive.stopMotor();
			}
			break;
		case 18://back up further to autozone
			if(!autonomousTimer.hasPeriodPassed(1.72)){//TODO 1.4
				drive.mecanumDrive_Cartesian(0, -0.575, 0, 0);//TODO cfg
			}else{
				autonomousState=6;
				drive.stopMotor();
			}
			break;
		case 19://turn right towards autozone
			if(!autonomousTimer.hasPeriodPassed(1.57)){
				drive.mecanumDrive_Cartesian(0, 0, -0.37 + autonomousTimer.get()*0.05, 0);//TODO cfg
				//if(autonomousTimer.get()<0.3){
				//	tilter.reverse();
				//}else{
				//	tilter.off();
				//}
			}else{
				drive.mecanumDrive_Cartesian(0, 0, -0.003, 0);//stop
				if(autonomousMode == 1 || autonomousMode == 2 || autonomousMode == 3){
					autonomousState = 6;
				}else if(autonomousMode == 0){
					autonomousState=5;
				}
			}
			break;
		case 6://drop crate
			if(!autonomousTimer.hasPeriodPassed(2)){//TODO cfg
				hooks.goDown(0.25);//TODO cfg
			}else{
				autonomousState++;
				hooks.stop();
			}
			break;
		case 7://move away from crate and can
			if(!autonomousTimer.hasPeriodPassed(0.6)){//TODO 0.6
				drive.mecanumDrive_Cartesian(0, -0.2, 0, 0);//TODO cfg
			}else{
				autonomousState = -1;
				drive.stopMotor();
			}
			break;    	
		}
    }
    
    
    //19
    public void autonomousPeriodic() {
    	//SmartDashboard.putNumber("AutoState", autonomousState);
    	switch(autonomousState){
		/*case 0://move to first object
			if(!autonomousTimer.hasPeriodPassed(0.1)){
				drive.mecanumDrive_Cartesian(0, 0.25F, 0, 0);//TODO cfg
			}else{
				autonomousState++;
				drive.stopMotor();
			}
			break;*/
    	case 0://plumb the tilter
    		if(tilter.straighten()){
    			autonomousState = 1;
    		}
    		break;
		case 1://pick up first object
			if(autonomousTimer.hasPeriodPassed(1.6)){//TODO .209 .6
				if(autonomousMode == 0){
					autonomousState = 11;
				}else if(autonomousMode == 1){
					autonomousState = 15;
				}else if(autonomousMode == 2){
					autonomousState = 16;
				}
				hooks.stop();
				autonomousTimer.reset();
			}else{
				hooks.move(0.5);
			}
			break;
		case 17://make room for tote
			if(hooks.moveOne(hooks::goUp,0.4)){//TODO .209 .55
				hooks.brake(true);
				autonomousState = 15;
				autonomousTimer.reset();
			}
			break;
		case 11://tilt back
			if(!autonomousTimer.hasPeriodPassed(0.8)){
				tilter.reverse();
			}else{
				if(autonomousMode == 0){
					autonomousState = 2;
				}else if(autonomousMode == 1 || autonomousMode == 2){
					autonomousState = 15;
				}
				tilter.off();
			}
			break;
		case 2://move to second object
			//if(!rangeFinder.isInRange()){
			if(!autonomousTimer.hasPeriodPassed(2)){//1.4
				drive.mecanumDrive_Cartesian(0, 0.16F, 0, 0);//0.185 - 0.17
			}else{
				autonomousState=10;
				drive.stopMotor();
			}
			break;
		case 15://Strafe to second object
			if(!autonomousTimer.hasPeriodPassed(1.3)){//1.4 1.9 1.8 1.63 1.47
				drive.mecanumDrive_Cartesian(0.5, 0.02, 0.035, 0);//0.5 0.04
			}else{

				if(autonomousMode == 0)
					autonomousState = 10;
				else if(autonomousMode == 1 || autonomousMode == 2){
					autonomousState = 13;
				}
				drive.stopMotor();
			}
			break;
		case 10://tilt forward
			if(!autonomousTimer.hasPeriodPassed(0.8)){
				tilter.forward();
			}else{
				autonomousState = 13;
				tilter.off();
			}
			break;
		case 13://move down
			if(!autonomousTimer.hasPeriodPassed(0.4)){//1.4
				hooks.goDown(0.2);
			}else{
				
				autonomousState=14;
				hooks.stop();
			}
			break;
		case 14://move forward to second object
			if(!autonomousTimer.hasPeriodPassed(1.4)){//1.4
				drive.mecanumDrive_Cartesian(0, 0.19F, 0, 0);//0.16
			}else{
				autonomousState=12;
				drive.stopMotor();
			}
			break;
		case 12://picking up object
			if(autonomousTimer.hasPeriodPassed(0.75)){//0.35 - 0.31 0.6 0.5
				//autonomousState++;
				hooks.stop();
				if(this.autonomousMode == 0){
					autonomousState = 4;
				}else if(autonomousMode == 1){
					autonomousState = 16;
				}else if(autonomousMode == 2)
					autonomousState = 18;
			}else{
				hooks.move(0.63);
			}
			break;
		case 3://pick up second object
			if(hooks.moveOne(hooks::goUp,0.4)){//0.35 - 0.31 0.6
				//autonomousState++;
				hooks.brake(true);
				if(this.autonomousMode == 0){
					autonomousState = 4;
				}else if(autonomousMode == 1){
					autonomousState = 16;
				}
				autonomousTimer.reset();
			}
			break;
		case 4://turn left towards autozone
			if(!autonomousTimer.hasPeriodPassed(1.57)){
				drive.mecanumDrive_Cartesian(0, 0, 0.37 - autonomousTimer.get()*0.05, 0);//TODO cfg
				//if(autonomousTimer.get()<0.3){
				//	tilter.reverse();
				//}else{
				//	tilter.off();
				//}
			}else{
				drive.mecanumDrive_Cartesian(0, 0, -0.003, 0);//stop
				if(autonomousMode == 1 || autonomousMode == 2){
					autonomousState = 6;
				}else if(autonomousMode == 0){
					autonomousState=5;
				}
			}
			break;
		case 5://move to autoZone
			if(!autonomousTimer.hasPeriodPassed(1.4)){//TODO cfg
				drive.mecanumDrive_Cartesian(0, 0.575, 0, 0);//TODO cfg
			}else{
				autonomousState=6;
				drive.stopMotor();
			}
			break;
		case 16://back up to autozone
			if(!autonomousTimer.hasPeriodPassed(1.35)){//TODO 1.4 1.25
				drive.mecanumDrive_Cartesian(0, -0.575, 0, 0);//TODO cfg
			}else{
				if(turnState == 0){
					autonomousState=6;
				}else if(turnState == 1){
					autonomousState = 4;
				}else if(turnState == 2){
					autonomousState = 19;
				}
				drive.stopMotor();
			}
			break;
		case 18://back up further to autozone
			if(!autonomousTimer.hasPeriodPassed(1.72)){//TODO 1.4
				drive.mecanumDrive_Cartesian(0, -0.575, 0, 0);//TODO cfg
			}else{
				autonomousState=6;
				drive.stopMotor();
			}
			break;
		case 19://turn right towards autozone
			if(!autonomousTimer.hasPeriodPassed(1.57)){
				drive.mecanumDrive_Cartesian(0, 0, -0.37 + autonomousTimer.get()*0.05, 0);//TODO cfg
				//if(autonomousTimer.get()<0.3){
				//	tilter.reverse();
				//}else{
				//	tilter.off();
				//}
			}else{
				drive.mecanumDrive_Cartesian(0, 0, -0.003, 0);//stop
				if(autonomousMode == 1 || autonomousMode == 2){
					autonomousState = 6;
				}else if(autonomousMode == 0){
					autonomousState=5;
				}
			}
			break;
		case 6://drop crate
			if(autonomousMode == 3){
				autonomousState = -1;
			}
			
			if(!autonomousTimer.hasPeriodPassed(2)){//TODO cfg
				hooks.goDown(0.25);//TODO cfg
			}else{
				autonomousState++;
				hooks.stop();
			}
			break;
		case 7://move away from crate and can
			if(!autonomousTimer.hasPeriodPassed(0.6)){//TODO 0.6
				drive.mecanumDrive_Cartesian(0, -0.2, 0, 0);//TODO cfg
			}else{
				autonomousState = -1;
				drive.stopMotor();
			}
			break;    	
		}
    }
    
    
    
    
    @Override
    public void testInit(){
    	//LiveWindow.setEnabled(true); //TODO determine if necessary
    	
    	
    }
    
    @Override
    public void testPeriodic() {
    	
    }
    
}
