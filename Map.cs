using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace DrRobot.JaguarControl
{
    public partial class Map : Form
    {
        private JaguarCtrl jaguarControl;
        public Map(JaguarCtrl jc)
        {
            jaguarControl = jc;
            InitializeComponent();
            this.MouseClick += new MouseEventHandler(click);
        }

        private int clickMode = 0;
        private JagPoint clickP;
        private int clickX = 0;
        private int clickY = 0;

        private void click(object sender, MouseEventArgs e)
        {
            Console.WriteLine("{0}:{1},{2}", clickMode, e.X, e.Y);
            switch(clickMode)
            {
                case 0:
                    clickX = e.X;
                    clickY = e.Y;
                    clickP = jaguarControl.scaleToActual(e.X, e.Y);
                    break;
                case 1:
                    double t = JagPoint.slopeTheta(clickP, jaguarControl.scaleToActual(e.X, e.Y));
                    jaguarControl.navigation.trajectory.addPoint(new JagPoint(clickP.x, clickP.y, t));
                    break;
            }
            clickMode = (clickMode +1) %2;
        }

        private void trackBar1_Scroll(object sender, EventArgs e)
        {
            jaguarControl.setTrackBarZoom(trackBar1.Value);
            mapScale.Text = trackBar1.Value.ToString();
        }

        private void Map_Load(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            Console.WriteLine(jaguarControl.navigation.trajectory.getMap());
        }
    }
}
