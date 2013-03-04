using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;

namespace DrRobot.JaguarControl
{
    public struct JagPoint
    {
        public double x;
        public double y;
        public double theta;
        public double time;

        public JagPoint(double X, double Y, double T = -1, double TIME = -1)
        {
            x = X;
            y = Y;
            theta = T;
            time = TIME;
        }

        public double distanceFrom(JagPoint a)
        {
            return JagPoint.distanceBetween(a,this);
        }

        public Boolean hasTheta()
        {
            return (theta != -1);
        }

        public String ToString()
        {
            return String.Format("{0:0.00}", x) + "," + String.Format("{0:0.00}", y) + "," + String.Format("{0:0.00}", theta);
        }

        static public double distanceBetween(JagPoint a, JagPoint b)
        {
            return Math.Sqrt(Math.Pow(a.x-b.x,2)+Math.Pow(a.y-b.y,2));
        }

        static public double slopeTheta(JagPoint a, JagPoint b)
        {
            return Math.Atan2(b.y - a.y, b.x - a.x);
        }
    }

    public class JagPath
    {
        public List<JagPoint> points;

        public JagPath()
        {
            points = new List<JagPoint>();
        }

        public void addPoint(JagPoint p)
        {
            points.Add(p);
        }
    }

    public class JagTrajectory
    {
        public static String circleTrajStr = "3,0,1.57075;1.500020485,2.598064384,2.617939667;-1.499959031,2.598099865,3.665129333;"
   + "-2.999999999,7.09608E-05,4.712319;-1.500081938,-2.598028903,5.759508667;1.499897576,-2.598135343,6.806698333;3,0,1.57075";

        public static String squareTrajStr = "3,3;3,-3;-3,-3;-3,3;3,3;3,-3;-3,-3;-3,3";

        public static String straightTrajStr = "1,0,0;2,0,0;3,0,0;4,0,0";

        public static String map1 = "1.00,0.00,0.00;2.00,0.00,0.00;3.00,0.00,0.00;4.00,0.00,0.00;4.83,0.47,0.79;5.11,1.17,1.49;4.92,1.97,2.65;3.86,2.25,-3.06;2.78,2.08,2.86;2.00,2.14,3.14;0.94,2.17,3.08;-0.31,2.22,3.06;-0.94,2.50,1.75;-1.03,3.17,0.84;-0.50,3.56,0.05;0.58,3.58,-0.24;1.67,3.39,-0.05;2.75,3.33,0.00;3.92,3.33,-0.05;4.92,3.25,-0.57;5.56,2.72,-1.07;5.86,2.11,-1.33;5.94,1.36,-1.75;5.81,0.53,-1.64";
        public static String esss = "0,0,0;0.5,0,-0.01;1,0.02,-0.01;1.54,0.02,-0.01;2,0,0.02;2.48,0.04,0.53;2.77,0.78,1.69;2.63,1.76,2.8;1.98,2.02,3.14;1.42,2.02,3.14;0.89,2.02,3.14";

        public List<JagPoint> points;
        int index = 0;
        public JagTrajectory()
        {
            points = new List<JagPoint>();
        }
        
        public void addPoint(JagPoint p)
        {
            points.Add(p);
        }

        public JagPoint getTargetPoint()
        {
            return points[index];
        }

        public Boolean empty()
        {
            return points.Count == 0;
        }

        public void nextPoint()
        {
            if(index +1 < points.Count)
            {
                index += 1;
            }
        }
        public double tangent()
        {
            if(getTargetPoint().hasTheta())
            {
                return getTargetPoint().theta;
            }
            int prev = index > 1 ? index-1 : index;
            int next = index < points.Count-1 ? index+1 : index;

            return JagPoint.slopeTheta(points[prev], points[next]);
        }

        public String getMap()
        {
            String str = "";
            foreach (JagPoint point in points)
            {
                str += point.ToString() + ";";
            }
            return str;
        }

        public Boolean isEnd()
        {
            if (index >= points.Count - 1)
            {
                return true;
            }
            else 
            { 
                return false; 
            }
        }

        public Boolean isWithinTheshhold(double threshhold, JagPoint a)
        {
            if (getTargetPoint().distanceFrom(a) < threshhold)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        public static JagTrajectory parseTxt(String str, Char splitChar = ';')
        {
            //"x,y,t;x,y,t;x,y,t"
            JagTrajectory jagTraj = new JagTrajectory();
            try
            {
                string[] points = str.Split(splitChar);
                foreach (string sp in points)
                {
                    string[] xyt = sp.Split(',');
                    double x = Double.Parse(xyt[0]);
                    double y = Double.Parse(xyt[1]);
                    double t = -1;
                    try{
                        t = Double.Parse(xyt[2]);
                    }catch{}

                    jagTraj.addPoint(new JagPoint(x, y, t));
                }
            }
            catch
            {
                throw;
            }
            return jagTraj;
        }


            

    }
}
