using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX, initialY, initialT;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

        public double aa, bb, cc, dd, ee;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089;
        public double robotRadius = 0.242;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        public double Kpho = 0.2;//1;
        public double Kalpha = 0.5;//2;//8
        public double Kbeta = -0.1;//-0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public double K_P = 1;//15;
        public double K_I = 0;//0;
        public double K_D = 3;//3;

        public double K_p = 25;//15;
        public double K_i = 0;//0;
        public double K_d = 0;//3;

        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        public JagTrajectory trajectory;
        public JagPath breadCrumbs;
        public int breadCrumbsInterval = 200;
        public int breadCrumbsCount = 0;
        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            x = 0;//initialX;
            y = 0;//initialY;
            t = 0;//initialT;

            // Initialize state estimates
            x_est = 0;//initialX;
            y_est = 0;//initialY;
            t_est = 0;//initialT;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            //InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            aa = 0;
            bb = 0;
            cc = 0;
            dd = 0;
            ee = 0;

            trajectory = new JagTrajectory();
            /*
            trajectory.addPoint(new jagPoint(1, 1, 1));
            trajectory.addPoint(new jagPoint(2, 2, 1));
            trajectory.addPoint(new jagPoint(3, 3, 1));
            trajectory.addPoint(new jagPoint(4, 4, 1));
            trajectory.addPoint(new jagPoint(4, 5, 1));*/
            trajectory = JagTrajectory.parseTxt(JagTrajectory.circleTrajStr);
            hasStartedTrackingTrajectory = false;

            breadCrumbs = new JagPath();
            breadCrumbs.addPoint(new JagPoint(x, y, t));
            breadCrumbsCount = 0;
        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(500);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();

            // Run infinite Control Loop
            while (runThread)
            {
                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                MotionPrediction();

                // Update the global state of the robot - x,y,t (lab 2)
                LocalizeRealWithOdometry();

                // Update the global state of the robot - x,y,t (lab 2)
                //LocalizeRealWithIMU();
                

                // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                LocalizeEstWithParticleFilter();


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {

                    // Check if we need to create a new trajectory
                    if (motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to 1meter from the wall. Otherwise, comment it out after lab 1. 
                    //WallPositioning();

                    // Drive the robot to a desired Point (lab 3)
                    if (jaguarControl.autoMode == jaguarControl.AUTO_TRACKTRAJ)
                    {
                        TrackTrajectory();
                    }
                    FlyToSetPoint();
                    breadCrumbsCount += deltaT;
                    if (breadCrumbsCount >= breadCrumbsInterval)
                    {
                        breadCrumbs.addPoint(new JagPoint(x, y, t));
                    }
                    // Follow the trajectory instead of a desired point (lab 3)
                    //TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds

                        desiredRotRateL = 1;

                        CalcMotorSignals();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }

        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        private short count = 1;

        public void CalcMotorSignals()
        {
            if (diffEncoderPulseR == 0 && diffEncoderPulseL == 0 && count < 20)
            {
                count += 1;
                return;
            }

            short zeroOutput = 16383;
            short maxPosOutput = 32767;
            // desiredRotRateR = 10;
            // desiredRotRateL = 10;
            double deltaTinS = (double)deltaT / 1000;
            // We will use the desiredRotRateRs to set our PWM signals
            desiredRotRateR = 600;
            desiredRotRateL = 600;

            double currentRotRateR = -jaguarControl.rightFrontWheelMotor.encodeSpeed * jaguarControl.rightFrontWheelMotor.encoderDir;//(diffEncoderPulseR / (count * deltaTinS));
            double currentRotRateL = jaguarControl.leftFrontWheelMotor.encodeSpeed * jaguarControl.leftFrontWheelMotor.encoderDir;//(diffEncoderPulseL / (count * deltaTinS));

            double cur_e_R = desiredRotRateR - currentRotRateR; //in units of rad per second
            double cur_e_L = desiredRotRateL - currentRotRateL;

            int e_dir_R = (int)(cur_e_R - e_R);
            int e_dir_L = (int)(cur_e_L - e_L);
            e_R = cur_e_R;
            e_L = cur_e_L;

            int maxErr = (int)(3000 / deltaT);


            //K_p = 0.1;//1
            K_i = 0;//12 / deltaT;//20
            K_d = 0;// 100.1;

            //Kpho = 1.5;
            //Kalpha = 8;//4
            //Kbeta = -0.8;//-1.0;


            u_R = K_p * e_R + K_i * e_sum_R + K_d * e_dir_R;
            u_L = K_p * e_L + K_i * e_sum_L + K_d * e_dir_L;

            motorSignalL = (short)(zeroOutput + u_L);//(zeroOutput + desiredRotRateL * 300);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);// (zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            e_sum_R = Math.Max(-maxErr, Math.Min(0.90 * e_sum_R + e_R * deltaT, maxErr));
            e_sum_L = Math.Max(-maxErr, Math.Min(0.90 * e_sum_L + e_L * deltaT, maxErr));

            Console.WriteLine(K_p + " " + cur_e_R + " "+ currentRotRateR);//"desired: " + desiredRotRateR.ToString() + " diffEncoderPulseR:" + diffEncoderPulseR + " diff/s:" + diffEncoderPulseR /(count* deltaTinS) + " cur_e_R: " + cur_e_R + " u_R:" + u_R + " motorSignalR:" + motorSignalR);

            aa = u_R;
            bb = diffEncoderPulseR;
            cc = diffEncoderPulseR / deltaTinS;
            dd = desiredRotRateR;
            ee = motorSignalR;

            count = 1;
        }

        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
            {
                // Setup Control
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(3, (short)K_P, (short)K_D, (short)K_I);
                jaguarControl.realJaguar.SetDcMotorVelocityControlPID(4, (short)K_P, (short)K_D, (short)K_I);

                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            }
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                String newData = time.ToString() + "," + aa + "," + bb+ "," + cc +"," + dd+ "," +ee;//;+ " " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************
            int dir = 1;

            double dx = desiredX - x_est;
            double dy = desiredY - y_est;

            double a = -1.0 * t_est + Math.Atan2(dy, dx);
            if(a < -Math.PI/2 || a>Math.PI/2){
                dir = -1;
                a = -1.0 * t_est + Math.Atan2(-dy, -dx);
            }

            double p = Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2)); //distance away from setpoint
            double b = -1.0 * t_est - a + boundAngle(desiredT, 2) ;
            
            double v = Kpho * p; //set velocity to v (m/s)
            v = Math.Min(maxVelocity, v);
            v = dir * v;
            double w = Kalpha * a + Kbeta * b; //set rotation to w

            aa = p;
            bb = v;
            cc = w;
            

            double w2 = 0.5 * (w + v / robotRadius); //Left rotation
            double w1 = 0.5 * (w - v / robotRadius); //Right rotation

            double phi1 = -1.0 * robotRadius * w1 / wheelRadius;
            double phi2 = robotRadius * w2 / wheelRadius;

            double pulsePerMeter = pulsesPerRotation / (wheelRadius * 2 * Math.PI);

            desiredRotRateL = (short)(phi1 * pulsePerMeter);
            desiredRotRateR = (short)(phi2 * pulsePerMeter);

            // System.Console.WriteLine("desiredRotRateL: " + desiredRotRateL + " desiredRotRateR: " + desiredRotRateR);

            //System.Console.WriteLine("x: " + x_est.ToString() + " y:" + y_est.ToString() + " t:" + t_est.ToString());
            //System.Console.WriteLine("dx:" + dx.ToString() + " dy:" + dy.ToString() + "p:" + p.ToString() + " a:" + a.ToString() + " b:" + b.ToString() + " v:" + v.ToString()
            //    + " w:" + w.ToString() + " w1:" + w1.ToString() + " w2:" + w2.ToString());

            //motorSignalL = phi1;
            //motorSignalR = phi2;
            // Put code here to calculate motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!


            // ****************** Additional Student Code: End   ************
        }


        public Boolean hasStartedTrackingTrajectory;
        public double trajThresh = 0.3;
        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            if (!hasStartedTrackingTrajectory)
            {
                JagPoint target = trajectory.getTargetPoint();
                desiredX = target.x;
                desiredY = target.y;
                desiredT = target.hasTheta() ? target.theta : 0;
            }
            if(!trajectory.isEnd() && trajectory.isWithinTheshhold(trajThresh,new JagPoint(x_est,y_est)))
            {
                trajectory.nextPoint();

                JagPoint target = trajectory.getTargetPoint();
                desiredX = target.x;
                desiredY = target.y;
                desiredT = target.hasTheta() ? target.theta : 0;
            }

        }

        // THis function is called to construct a collision-free trajectory for the robot to follow
        private void PRMMotionPlanner()
        {

        }


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public void MotionPrediction()//CWiRobotSDK* m_MOTSDK_rob)
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.



            diffEncoderPulseR = -(currentEncoderPulseR - lastEncoderPulseR);
            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;

            // If rollover occurs, this will measure the correct encoder difference
            if (Math.Abs(diffEncoderPulseR) > 0.5 * encoderMax)
            {
                if (diffEncoderPulseR < 0)
                {
                    diffEncoderPulseR = -encoderMax - 1 - diffEncoderPulseR;
                }
                if (diffEncoderPulseR > 0)
                {
                    diffEncoderPulseR = encoderMax + 1 - diffEncoderPulseR;
                }
            }
            if (Math.Abs(diffEncoderPulseL) > 0.5 * encoderMax)
            {
                if (diffEncoderPulseL < 0)
                {
                    diffEncoderPulseL = -encoderMax - 1 - diffEncoderPulseL;
                }
                if (diffEncoderPulseL > 0)
                {
                    diffEncoderPulseL = encoderMax + 1 - diffEncoderPulseL;
                }
            }


            // distance = r*theta where r=wheelRadius and theta=2*pi*encoderMeasurement/pulsesPerRevolution
            wheelDistanceR = wheelRadius * 2 * Math.PI * diffEncoderPulseR / pulsesPerRotation;
            wheelDistanceL = wheelRadius * 2 * Math.PI * diffEncoderPulseL / pulsesPerRotation;

            // Distance travelled is the average of the left and right wheel distances
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);

            //Save encoder value for next loop iteration
            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;

            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi

            // Update the actual
            x = x + distanceTravelled * Math.Cos(t + angleTravelled / 2);
            y = y + distanceTravelled * Math.Sin(t + angleTravelled / 2);
            t = t + angleTravelled;
            //Console.WriteLine(t);

            if (t > Math.PI)
            {
                t = t - 2 * Math.PI;
            }
            if (t < -1 * Math.PI)
            {
                t = t + 2 * Math.PI;
            }

            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()//CWiRobotSDK* m_MOTSDK_rob)
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }


        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            x_est = x;
            y_est = y;
            t_est = t;

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF




            // ****************** Additional Student Code: End   ************

        }
        #endregion

        private double boundAngle(double theta, double sthPi)
        {
            //if sthpi is 2, then it bounds between 0 and 2pi
            if (theta <= sthPi * Math.PI && theta >= sthPi * Math.PI - 2 * Math.PI)
            {
                return theta;
            }
            if (theta > sthPi * Math.PI)
            {
                theta -= Math.PI;
            }
            else if (theta < sthPi * Math.PI - 2 * Math.PI)
            {
                theta += Math.PI;
            }
            return boundAngle(theta, sthPi);
        }

    }
}
