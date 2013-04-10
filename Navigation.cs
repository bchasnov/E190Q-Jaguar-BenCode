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

        private DateTime globalLoopTime;

        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX=0, initialY=-2.8, initialT=1.57;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double desiredX, desiredY, desiredT;

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
        private double Kpho = 1;
        private double Kalpha = 2;//8
        private double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public short K_P = 15;//15;
        public short K_I = 0;//0;
        public short K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;

        public JagTrajectory trajectory;
        public JagPath breadCrumbs;
        public int breadCrumbsInterval = 200;
        public int breadCrumbsCount = 0;


        // PF Variables
        public Map map;
        public Particle[] particles;
        //public Particle[] propagatedParticles;
        public Particle[] tempParticles;
        public int tempParticlesCount = 0;
        public int numParticles = 1000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;
        private int newLaserCounter = 0;


        private int redistributeFactor = 4;

        public Boolean correctionOverrideEnabled = false;
        public Boolean correctionOverride = false;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }

        #endregion


        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];
            //propagatedParticles = new Particle[numParticles];
            tempParticles = new Particle[numParticles * redistributeFactor];
            // Create particles
            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                //propagatedParticles[i] = new Particle();
            }

            for (int i = 0; i < numParticles * redistributeFactor; i++)
            {
                tempParticles[i] = new Particle();
            }

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
            x = initialX;
            y = initialY;
            t = initialT;

            // Initialize state estimates
            x_est = initialX;
            y_est = initialY;
            t_est = initialT;

            // Set desired state
            desiredX = initialX;
            desiredY = initialY;
            desiredT = initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            trajectory = new JagTrajectory();
            trajectory = JagTrajectory.parseTxt(trajMap);
            hasStartedTrackingTrajectory = false;

            breadCrumbs = new JagPath();
            breadCrumbs.addPoint(new JagPoint(x, y, t));
            breadCrumbsCount = 0;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

        }

        public String trajMap = "0,0,0";

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
                globalLoopTime = DateTime.Now;

                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).

                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                Boolean moved = MotionPrediction();

                if (moved || correctionOverride)
                {
                    Console.WriteLine("Tick");
                    // Update the global state of the robot - x,y,t (lab 2)
                    LocalizeRealWithOdometry();
                    // Update the global state of the robot - x,y,t (lab 2)
                    //LocalizeRealWithIMU();

                    //Console.WriteLine(">"+x+","+y+","+t+","+map.GetClosestWallDistance(x, y, t));

                    // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                    LocalizeEstWithParticleFilter();
                }


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


                    // Follow the trajectory instead of a desired point (lab 3)
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

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
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
                while ((DateTime.Now- globalLoopTime).Milliseconds < deltaT)
                {

                    Thread.Sleep(1);
                }
                //Thread.Sleep(deltaT); //not sure if this works anymore..... -wf
            }
        }

        #region Sensors and signals
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

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }

                    laserCounter = 0;
                    newLaserData = true;
                }
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
        public void CalcMotorSignals()
        {
            short zeroOutput = 16383;
            short maxPosOutput = 32767;



            motorSignalL = (short)(zeroOutput);
            motorSignalR = (short)(zeroOutput);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

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
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion
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
                double weights = 0;
                for (int i = 0; i < numParticles; i++)
                {
                    weights += particles[i].w;
                }
                weights = weights / numParticles;
                String newData = time.ToString() + " " + Math.Sqrt(Math.Pow(x - x_est, 2) + Math.Pow(y - y_est, 2)) + " " + avgWeight + " " + x + " " + y + " " + t + " " + x_est + " " + y_est + " " + t_est;


                                  //weights;//;" " + x.ToString() + " " + y.ToString() + " " + t.ToString() ;

                logFile.WriteLine(newData);
            }
        }
        #endregion


        #region Misc. functions
        static public double boundAngle(double theta, double sthPi)
        {
            //if sthpi is 2, then it bounds between 0 and 2pi
            if (theta <= sthPi * Math.PI && theta >= sthPi * Math.PI - 2 * Math.PI)
            {
                return theta;
            }
            if (theta > sthPi * Math.PI)
            {
                theta -= 2 * Math.PI;
            }
            else if (theta < sthPi * Math.PI - 2 * Math.PI)
            {
                theta += 2 * Math.PI;
            }
            return boundAngle(theta, sthPi);
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

        public Boolean initFlyToSetPoint;
        public int dir;

        public void setSetPoint(double xx, double yy, double tt)
        {
            desiredX = xx;
            desiredY = yy;
            desiredT = tt;
            resetFlyToSetPoint();
        }

        public void resetFlyToSetPoint()
        {
            initFlyToSetPoint = true;
        }

        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate desiredRotRateR and 
            // desoredRotRateL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            double dx = desiredX - x;
            double dy = desiredY - y;

            double a = -1.0 * t_est + Math.Atan2(dy, dx);
            a = boundAngle(a, 1);
            if (initFlyToSetPoint)
            {//rear facing
                if (a < -Math.PI / 2 || a > Math.PI / 2)
                {
                    dir = -1;
                }
                else { dir = 1; }
                //Console.WriteLine("direction: " + dir);
                initFlyToSetPoint = false;
            }
            a = -1.0 * t_est + Math.Atan2(dir * dy, dir * dx);
            a = boundAngle(a, 1);
            double p = Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2)); //distance away from setpoint
            double b = -1.0 * t_est - a + boundAngle(desiredT, 1);
            b = boundAngle(b, 1);

            //Console.WriteLine("desired: {0} 2: {1} 1: {2}", desiredT, boundAngle(desiredT, 2), boundAngle(desiredT, 1));
            double v = Kpho * p; //set velocity to v (m/s)
            v = Math.Min(maxVelocity, v);

            if (!trajectory.isEnd())
            {
                v = maxVelocity;
            }

            v = dir * v;
            double w = Kalpha * a + Kbeta * b; //set rotation to w

            double w2 = 0.5 * (w + v / robotRadius); //Left rotation
            double w1 = 0.5 * (w - v / robotRadius); //Right rotation

            double phi1 = -1.0 * robotRadius * w1 / wheelRadius;
            double phi2 = robotRadius * w2 / wheelRadius;

            double pulsePerMeter = pulsesPerRotation / (2 * Math.PI * robotRadius);

            desiredRotRateL = (short)(phi1 * pulsePerMeter);
            desiredRotRateR = (short)(phi2 * pulsePerMeter);


            // ****************** Additional Student Code: End   ************
        }


        public Boolean hasStartedTrackingTrajectory;
        public double trajThresh = 0.5;
        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            if (trajectory.empty())
                return;

            if (!hasStartedTrackingTrajectory)
            {
                JagPoint target = trajectory.getTargetPoint();
                setSetPoint(target.x, target.y, trajectory.tangent());
                hasStartedTrackingTrajectory = true;
            }
            if (!trajectory.isEnd() && trajectory.isWithinTheshhold(trajThresh, new JagPoint(x, y)))
            {
                trajectory.nextPoint();

                JagPoint target = trajectory.getTargetPoint();
                desiredX = target.x;
                desiredY = target.y;
                desiredT = target.hasTheta() ? target.theta : 0;

                resetFlyToSetPoint();
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
        public Boolean MotionPrediction()
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

            if (diffEncoderPulseL == 0 && diffEncoderPulseR == 0)
                return false;

            return true;
            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
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

            t = boundAngle(t, 1);



            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }

        DateTime prevTime = DateTime.Now;
        void ttime(string str)
        {
            DateTime tmpTime = prevTime;
            prevTime = DateTime.Now;
            Console.WriteLine(">>"+str);
            Console.WriteLine(">>t=" + (prevTime - tmpTime).Milliseconds);
        }

        double avgWeight;
        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            

            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x_est, y_est, t_est using a PF

            double p_wheelDistanceR = 0;
            double p_wheelDistanceL = 0;
            double p_distanceTravelled = 0;
            double p_angleTravelled = 0;

            //Propagate particles with randomness
            Console.WriteLine("prediction "+time);
            ttime("start");
            for (int i = 0; i < numParticles; i++)
            {
                // distance = r*theta where r=wheelRadius and theta=2*pi*encoderMeasurement/pulsesPerRevolution
                p_wheelDistanceL = wheelDistanceL + wheelDistanceL* RandomGaussian() * K_wheelRandomness;
                p_wheelDistanceR = wheelDistanceR + wheelDistanceR * RandomGaussian() * K_wheelRandomness;

                // Distance travelled is the average of the left and right wheel distances
                p_distanceTravelled = (p_wheelDistanceL + p_wheelDistanceR) / 2;
                p_angleTravelled = (p_wheelDistanceR - p_wheelDistanceL) / (2 * robotRadius);

                particles[i].x = particles[i].x + p_distanceTravelled * Math.Cos(particles[i].t + p_angleTravelled / 2);
                particles[i].y = particles[i].y + p_distanceTravelled * Math.Sin(particles[i].t + p_angleTravelled / 2);
                particles[i].t = particles[i].t + p_angleTravelled;

                particles[i].t = boundAngle(particles[i].t, 1);

            }
            ttime("prediction");

            Console.WriteLine(newLaserData + "<><>");
            if (newLaserData)
            {
                newLaserCounter++;
                //newLaserData = false;
                Console.WriteLine(newLaserCounter + "<<<<<");
            }

            if (newLaserCounter > 2 && ((!correctionOverrideEnabled) || (correctionOverrideEnabled && correctionOverride)))
            {
                newLaserCounter = 0;
                Console.WriteLine("Correction Step");
                correctionOverride = false;
                //calculate unnormalized weight
                for (int i = 0; i < numParticles; i++)
                {
                    CalculateWeight(i);
                }
                ttime("calculate weight");
                //calculate normalized weight
                double maxWeight = 0.01;
                
                avgWeight = 0;
                for (int i = 0; i < numParticles; i++)
                {
                    maxWeight = Math.Max(particles[i].w, maxWeight);
                    avgWeight += particles[i].w;
                }
                avgWeight = avgWeight / numParticles;
                Console.WriteLine(avgWeight);
                for (int i = 0; i < numParticles; i++)
                {
                    particles[i].w = particles[i].w / maxWeight;
                    //Console.Write(propagatedParticles[i].w+" ");
                }
                //LogData();
                Console.WriteLine();

                ttime("normalize");

                //correction step
                //udacityResample();
                approximateRedistribute();
                //exactRedistribute();

                ttime("resample");

            }
            else { Console.WriteLine("skipped correction"); }

            //average to find the current robot state
            for (int i = 0; i < numParticles; i++)
            {
                x_est += particles[i].x;
                y_est += particles[i].y;
                t_est += particles[i].t;
            }
            x_est = x_est / numParticles;
            y_est = y_est / numParticles;
            t_est = t_est / numParticles;

            // ****************** Additional Student Code: End   ************

        }

        private void udacityResample()
        {
            tempParticlesCount = 0;

            int index = (int)(myRandom.NextDouble()*(numParticles-1));
            double b = 0;
            double wmax = 0;
            for(int i = 0; i < numParticles; i++)
            {
                wmax = Math.Max(wmax, particles[i].w);
            }

            for (int i = 0; i < numParticles; )
            {
                b += myRandom.NextDouble() * 2 * wmax;
                //Console.Write(b+" "+wmax);

                if (particles[i].w < b)
                {
                    b -= particles[i].w;
                    index += 1;
                    index = index % numParticles;
                }
                else
                {
                    copyParticleIntoTemp(index, 1);
                    //Console.Write(index);
                    //Console.Write(" ");
                    i++;
                }

            }
            for (int i = 0; i < numParticles; i++)
            {
                particles[i].x = tempParticles[i].x;
                particles[i].y = tempParticles[i].y;
                particles[i].t = tempParticles[i].t;
                particles[i].w = tempParticles[i].w;
            }

        }


        private void exactRedistribute()
        {
            double totalWeight = 0;
            double sumWeight = 0;
            for (int i = 0; i < numParticles; i++)
            {
                totalWeight += particles[i].w;
            }
            double r;
            int j = 0;
            for (int i = 0; i < numParticles; i++)
            {
                r = myRandom.NextDouble() * totalWeight;
                j = 0;
                sumWeight = particles[i].w;
                while (sumWeight < r)
                {
                    j++;
                    sumWeight += particles[i].w;
                }
            }
        }

        private void approximateRedistribute()
        {
            tempParticlesCount = 0;
            double[] c = { 0.000001, 0.2, 0.4, 0.7, 1 };
            for (int i = 0; i < numParticles; i++)
            {
                /*if (particles[i].w < c[0])
                {
                    particles[i].x = map.minX + myRandom.NextDouble() * (map.maxX - map.minX);
                    particles[i].y = map.minY + myRandom.NextDouble() * (map.maxY - map.minY);
                    particles[i].t = myRandom.NextDouble() * Math.PI * 2;

                    copyParticleIntoTemp(i, 1);
                    continue;
                }*/
                if (particles[i].w < c[1])
                {
                    copyParticleIntoTemp(i, 1);
                    continue;
                }
                if (particles[i].w < c[2])
                {
                    copyParticleIntoTemp(i, 2);
                    continue;
                }
                if (particles[i].w < c[3])
                {
                    copyParticleIntoTemp(i, 3);
                    continue;
                }
                if (particles[i].w <= c[4])
                {
                    copyParticleIntoTemp(i, 4);
                }
            }
            //fillTempWithRandomness();

            for (int i = 0; i < numParticles; i++)
            {
                int r = (int)(tempParticlesCount * myRandom.NextDouble());
                particles[i].x = tempParticles[r].x;
                particles[i].y = tempParticles[r].y;
                particles[i].t = tempParticles[r].t;
            }
            tempParticlesCount = 0;
        }

        private void copyParticleIntoTemp(int p, int n)
        {
            for (int i = 0; i < n; i++)
            {
                tempParticles[tempParticlesCount].x = particles[p].x;
                tempParticles[tempParticlesCount].y = particles[p].y;
                tempParticles[tempParticlesCount].t = particles[p].t;
                tempParticlesCount += 1;
            }
            
        }

        private void fillTempWithRandomness()
        {
            double percentRandomness = 0.1;
            int numRandomness = (int)(tempParticlesCount*percentRandomness);
            while (tempParticlesCount < numParticles*4 && numRandomness > 0)//*(redistributeFactor-2))
            {
                /*
                tempParticles[tempParticlesCount].x = x_est + RandomGaussian() * 0.1;
                tempParticles[tempParticlesCount].y = y_est + RandomGaussian() * 0.1;
                tempParticles[tempParticlesCount].t = t_est + RandomGaussian() * Math.PI / 100;
                */

                tempParticles[tempParticlesCount].x = map.minX + myRandom.NextDouble() * (map.maxX - map.minX);
                tempParticles[tempParticlesCount].y = map.minY + myRandom.NextDouble() * (map.maxY - map.minY);
                tempParticles[tempParticlesCount].t = myRandom.NextDouble() * Math.PI * 2;

                tempParticlesCount++;
                numRandomness--;
            }
        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {
            long expectedMeasurement = 0;
            long weight = 0;
            int n = 0;
            int sigma = 100000;

	        // ****************** Additional Student Code: Start ************
            for (int i = 0; i < LaserData.Length; i+=18)
            {
                expectedMeasurement = (long)(1000 * map.GetClosestWallDistance(particles[p].x, particles[p].y, particles[p].t - 1.570796327
 + laserAngles[i]));
                
                //n++;
                //weight *= (1.0 / Math.Sqrt(2 * Math.PI * sigma)) * Math.Exp(-Math.Pow(expectedMeasurement - LaserData[i], 2) / (Math.Pow(sigma, 2)*2));//Math.Pow(expectedMeasurement - LaserData[i],2)/1000;


                if (expectedMeasurement > 0 && LaserData[i] > 0)
                {
                    n++;
                    weight += (expectedMeasurement - LaserData[i]) * (expectedMeasurement - LaserData[i]);
                }

                if (p == 0)
                {
                    //Console.WriteLine(p + "," + expectedMeasurement + "," + LaserData[i] + "," + weight);
                }

            }
            if(n>0)
                particles[p].w = Math.Exp(-(weight / n) / sigma);

        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles() {


	        // Set particles in random locations and orientations within environment
	        for (int i=0; i< numParticles; i++){

		        // Either set the particles at known start position [0 0 0],  
		        // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
    		        SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
		            SetStartPos(i);
	        }
            
        }



        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        Random myRandom = new Random();
        

        void SetRandomPos(int p){

	        // ****************** Additional Student Code: Start ************

	        // Put code here to calculated the position, orientation of 
            // particles[p]. Feel free to use the random.NextDouble() function. 
	        // It might be helpful to use boundaries defined in the
	        // Map.cs file (e.g. map.minX)
            
            particles[p].x = map.minX + myRandom.NextDouble()*(map.maxX - map.minX);
            particles[p].y = map.minY + myRandom.NextDouble() * (map.maxY - map.minY);
            particles[p].t = RandomGaussian() * 0.1 + initialT;
            /*double radius = 2;
            particles[p].x = RandomGaussian() * radius + initialX;
            particles[p].y = RandomGaussian() * radius + initialY;
            particles[p].t = RandomGaussian() * 0.1 + initialT;
            */
            particles[p].w = 1 / numParticles;

            // ****************** Additional Student Code: End   ************
        }




        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p){
	        particles[p].x = initialX;
	        particles[p].y = initialY;
	        particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
	        double U1, U2, V1=0, V2;
	        double S = 2.0;
	        while(S >= 1.0) 
	        {
		        U1 = random.NextDouble();
                U2 = random.NextDouble();
		        V1 = 2.0*U1-1.0;
		        V2 = 2.0*U2-1.0;
		        S = Math.Pow(V1,2) + Math.Pow(V2,2);
	        }
	        double gauss = V1*Math.Sqrt((-2.0*Math.Log(S))/S);
	        return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
	        if (a>0)
                return 1.0;
	        else if (a<0)
                return -1.0;
	        else
                return 0.0;
        }

        #endregion

    }
}
