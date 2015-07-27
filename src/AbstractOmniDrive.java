	import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.Date;

	import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.TachoMotorPort;
import lejos.nxt.addon.LnrActrFirgelliNXT;
import lejos.nxt.addon.MMXRegulatedMotor;
import lejos.nxt.addon.NXTMMX;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import lejos.robotics.EncoderMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.RegulatedMotorListener;

	public abstract class AbstractOmniDrive {

		protected RegulatedMotor motor1;
		protected RegulatedMotor motor2;
		protected RegulatedMotor motor3;
		protected RegulatedMotor motor4;
		protected LnrActrFirgelliNXT linearActuator;


		protected  void setMaxSpeed(){
			MAX_SPEED = Math.min(Math.min(motor1.getMaxSpeed(), motor2.getMaxSpeed()), Math.min(motor3.getMaxSpeed(), motor4.getMaxSpeed()));
		}
		
		/**
		 * Utility method to get Motor instance from string (A, B or C)
		 */
		public static RegulatedMotor getMotor(String motor) {
			NXTMMX mux = new NXTMMX(SensorPort.S1);
			if (motor.equals("A"))
				return Motor.A;
			else if (motor.equals("B"))
				return Motor.B;
			else if (motor.equals("C"))
				return Motor.C;
			else if (motor.equals("1"))
				return new MMXRegulatedMotor(mux, NXTMMX.MMX_MOTOR_1);
			else if (motor.equals("2"))
				return new MMXRegulatedMotor(mux, NXTMMX.MMX_MOTOR_2);
			else
				return null;
		}

		public static LnrActrFirgelliNXT getLinAcc(){
			return new LnrActrFirgelliNXT(MotorPort.C);
		}
		
		protected float topSpeed = 0;
		protected float vx = 0;
		protected float vy = 0;
		protected float turningspeed = 0;
		protected int status = ResponseCode.OK.value;
		
		protected float MAX_SPEED;

		protected void stopAllMotors() {
			motor1.stop(true);
			motor2.stop(true);
			motor3.stop(true);
			motor4.stop(true);
		}

		private void startMotor(RegulatedMotor motor, double motorSpeed) {
			motor.setSpeed((int) Math.round(this.topSpeed * motorSpeed));
			if (motorSpeed > 0) {
				motor.forward();
			} else {
				motor.backward();
			}
		}

		/**
		 * Estabish bluetooth connection to mission control
		 */
		public void connect() {
			LCD.clear();
			LCD.drawString("Waiting", 0, 0);
			connection = Bluetooth.waitForConnection(); // this method is very
														// patient.
			LCD.clear();
			LCD.drawString("Connected", 0, 0);
			dataIn = connection.openDataInputStream();
			dataOut = connection.openDataOutputStream();
			Sound.beepSequence();
		}

		/**
		 * connect and wait for orders
		 */
		protected void go() {
			connect();
			readThread.start();
			sendThread.start();
			stallDetector.start();
			while (true) {
				if (Button.ENTER.isDown()) {
					readThread.interrupt();
					try {
						dataIn.close();
						dataOut.close();
					} catch (IOException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					try {
						this.wait(500);
					} catch (Exception e) {
						e.printStackTrace();
					}
					connection.close();
					stopAllMotors();
					System.exit(0);
				}
			}
		}

		protected float tempv1;
		protected float tempv2;
		protected float tempv3;
		protected float tempv4;
		protected float overflow = 0;
		
		protected enum Command{
			GETSTATUS(1),
			DRIVE(2),
			LIFTINGARM(3),
			DISCONNECT(4);		
			
			private int value;
			
			private Command(int value){
				this.value= value;
			}
		}
		
		
		protected enum ResponseCode{
			OK(0),
			ERROR(-1),
			SKIPINVALIDCOMMAND(3),
			DISCONNECT(4),
			LOWBATTERY(5),
			MOTORSTALLED(6);
					
			private int value;
			
			private ResponseCode(int value){
				this.value= value;
			}
		}
		
		protected Thread readThread = new Thread(){
			
			@Override
			public void run(){
				while(!isInterrupted())
					readData();
			}
			
			/**
			 * decode incoming messages and handle motors accordingly
			 * 
			 * {int: command, ...}
			 * 
			 * {GETSTATUS = 1}
			 * 
			 * 
			 * {DRIVE = 2, float: topSpeed in [0, Float.MaxValue], float: vx in [-1,1], float: vy in [-1,1], float: turn in [-1,1]}
			 * 
			 * 
			 * {LIFTINGARM = 3, float: position in [0,90]}
			 * 
			 * 
			 * {DISCONNECT = 4}
			 * 
			 * 
			 * response:
			 * {int: status, int: actualSpeed}
			 * 
			 * status: 0: ok; 1: skip invalid command; -1: error;
			 * 
			 * actualSpeed in [0, infinity]
			 * 
			 * 
			 * Wheels:
			 * Front
			 * ---1---
			 * |  |  |
			 * 2- - -3
			 * |  |  |
			 * ---4---
			 * Back 
			 * see: http://www.firstroboticscanada.org/main/wp-content/uploads/Omnidirectional-Drive-Systems.pdf
			 */
		protected void readData() {
			try {
				if (dataIn.available() > 0) {
					if (dataIn.readInt() == Command.DRIVE.value) {
						handleDriveCommand();
					} else if (dataIn.readInt() == Command.GETSTATUS.value) {
						sendStatus();
					} else if (dataIn.readInt() == Command.LIFTINGARM.value) {
						handleLiftingarmCommand();
					} else if (dataIn.readInt() == Command.DISCONNECT.value) {
						disconnect();
					}
				} else {
					try {
						Thread.sleep(0);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			} catch (IOException e) {
				System.out.println("Read exception " + e);
			}
		}

			private void disconnect() {
				status = ResponseCode.DISCONNECT.value;
				send = true;
				connection.close();
				System.exit(0);
			}

			private void handleLiftingarmCommand() throws IOException {
				float angle = dataIn.readFloat();
				driveLiftingarmToAngle(angle);
				status = ResponseCode.OK.value;
				send = true;
			}

			private void sendStatus() {
				send = true;
			}

			private void handleDriveCommand() throws IOException {
				topSpeed = dataIn.readFloat();
				vx = dataIn.readFloat();
				vy = dataIn.readFloat();
				turningspeed = dataIn.readFloat();
				boundsCheck(vx, -1, 1);
				boundsCheck(vy, -1, 1);
				boundsCheck(turningspeed, -1, 1);
				boundsCheck(topSpeed, 0, MAX_SPEED);
				LCD.clear();
				if(topSpeed > MAX_SPEED)
					topSpeed = MAX_SPEED;
				LCD.drawString(Float.toString(topSpeed), 0,
						3);
				LCD.drawString(Float.toString(vx), 0, 4);
				LCD.drawString(Float.toString(vy), 0, 5);
				LCD.drawString(Float.toString(turningspeed), 0, 6);
				LCD.drawString(Float.toString(MAX_SPEED), 0,
						7);
				status = ResponseCode.OK.value;
				send = true;
				if ((vx != 0 || vy != 0 || turningspeed != 0) && topSpeed > 0) {
					setMotorSpeed( vx,  vy,  turningspeed);
					overflow = Math.max(
							Math.max(Math.abs(tempv1), Math.abs(tempv2)),
							Math.max(Math.abs(tempv3), Math.abs(tempv4)));
					if (overflow > 1) {
						tempv1 /= overflow;
						tempv2 /= overflow;
						tempv3 /= overflow;
						tempv4 /= overflow;
					}
					startMotor(motor1, tempv1);
					startMotor(motor2, tempv2);
					startMotor(motor3, tempv3);
					startMotor(motor4, tempv4);
				} else {
					stopAllMotors();
				}
			}

			
		};
		protected boolean send = false;

		protected void boundsCheck(float value, float min, float max) {
			if (value > max)
				value = max;
			if (value < min)
				value = min;
		}
		
		protected abstract void driveLiftingarmToAngle(float angle);

		protected abstract void setMotorSpeed(float vx2, float vy2, float turningspeed2);

		protected Thread sendThread = new Thread() {

			@Override
			public void run() {
				while (!isInterrupted())
					sendData();
			}

			protected void sendData() {
				if (send) {
					send = false;
					try {
						dataOut.writeInt(status);
						dataOut.writeInt(Math.round(topSpeed));
						dataOut.flush();
					} catch (IOException e) {
						LCD.clear();
						LCD.drawString("ERROR Sending to host", 0, 3);
					}
				} else {
					try {
						Thread.sleep(0);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}

		};
		
		protected boolean running = true; 
		
		protected boolean stalled = false;
		
		protected Thread  stallDetector = new Thread(){
			public void run(){
				while(running){
					try {
						Thread.sleep(100);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					if(motor1.isStalled()){
						LCD.drawString("Motor 1 stalled", 0, 0);
						startMotor(motor1, tempv1);
					}
					if(motor2.isStalled()){
						LCD.drawString("Motor 2 stalled", 1, 0);
						startMotor(motor2, tempv2);
					}
					if(motor3.isStalled()){
						LCD.drawString("Motor 3 stalled", 2, 0);
						startMotor(motor3, tempv3);
					}
					if(motor4.isStalled()){
						LCD.drawString("Motor 4 stalled", 3, 0);
						startMotor(motor4, tempv4);
					}
				}
			}
		};
		
		protected BTConnection connection;
		protected DataInputStream dataIn;
		protected DataOutputStream dataOut;
	}


