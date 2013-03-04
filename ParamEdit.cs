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
    public partial class ParamEdit : Form
    {
        JaguarCtrl jaguar;
        public ParamEdit(JaguarCtrl jc)
        {
            jaguar = jc;
            InitializeComponent();
            button1.PerformClick();
        }

        private void linkParams()
        {


        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.K_P = Double.Parse(txt.Text);
                jaguar.navigation.InitiateVelControl();
            }
        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            K_p.Text = jaguar.navigation.K_P.ToString();
            K_i.Text = jaguar.navigation.K_I.ToString();
            K_d.Text = jaguar.navigation.K_D.ToString();
            Kalpha.Text = jaguar.navigation.Kalpha.ToString();
            Kpho.Text = jaguar.navigation.Kpho.ToString();
            Kbeta.Text = jaguar.navigation.Kbeta.ToString();
            trajThresh.Text = jaguar.navigation.trajThresh.ToString();

        }

        private void ParamEdit_Load(object sender, EventArgs e)
        {

        }

        private void Kpho_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.Kpho = Double.Parse(txt.Text);
            }
        }

        private void Kalpha_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.Kalpha = Double.Parse(txt.Text);
            }
        }

        private void Kbeta_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.Kbeta = Double.Parse(txt.Text);
            }
        }

        private void trajThresh_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.trajThresh = Double.Parse(txt.Text);
            }
        }

        private void textBox7_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.msR = (short)Double.Parse(txt.Text);
            }
        }

        private void label8_Click(object sender, EventArgs e)
        {

        }

        private void K_i_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.K_I = Double.Parse(txt.Text);
                jaguar.navigation.InitiateVelControl();
            }
        }

        private void K_d_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.K_D = Double.Parse(txt.Text);
                jaguar.navigation.InitiateVelControl();
            }
        }

        private void msL_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.msL = (short)Double.Parse(txt.Text);
                jaguar.navigation.InitiateVelControl();
            }
        }

        private void checkBox1_CheckedChanged(object sender, EventArgs e)
        {
            CheckBox cb = sender as CheckBox;
            if (cb.Checked)
            {
                jaguar.navigation.overrideMotorSignals = true;
            }
            else
            {
                jaguar.navigation.overrideMotorSignals = false;
            }
        }


    }
}
