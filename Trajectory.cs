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
            return "x: " + x + "y: " + y;
        }

        static public double distanceBetween(JagPoint a, JagPoint b)
        {
            return Math.Sqrt(Math.Pow(a.x-b.x,2)+Math.Pow(a.y-b.y,2));
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
   + "-2.999999999,7.09608E-05,4.712319;-1.500081938,-2.598028903,5.759508667;1.499897576,-2.598135343,6.806698333";

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

        public void nextPoint()
        {
            if(index +1 < points.Count)
            {
                index += 1;
            }
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
                    double t = Double.Parse(xyt[2]);

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
