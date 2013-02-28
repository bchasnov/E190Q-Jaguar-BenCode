using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    class Point
    {
        public double x;
        public double y;
        public double theta;
        public double time;

        Point(double X, double Y, double T = null, double TIME = null)
        {
            x = X;
            y = Y;
            theta = T;
            time = TIME;
        }

        public double distanceFrom(Point a)
        {
            return Point.distanceBetween(a,this);
        }

        static public double distanceBetween(Point a, Point b)
        {
            return Math.Sqrt(Math.Pow(a.x-b.x,2)+Math.Pow(a.y-b.y,2));
        }
    }
    class Trajectory
    {
        List<Point> points;
        int index = 0;
        Trajectory()
        {
            points = new List<Point>();
        }
        
        public void addPoint(Point p)
        {
            points.Add(p);
        }

        public Point getTargetPoint()
        {
            return points[index];
        }

        public void nextPoint()
        {
            index += 1;
        }

        public Boolean isWithinTheshhold(double threshhold, Point a)
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

    }
}
