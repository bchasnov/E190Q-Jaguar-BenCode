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
        }

        private void linkParams()
        {


        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {
            TextBox txt = sender as TextBox;
            if (txt != null)
            {
                jaguar.navigation.K_p = Double.Parse(txt.Text);
            }
        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            K_p.Text = jaguar.navigation.K_p.ToString();
            K_i.Text = jaguar.navigation.K_i.ToString();
            K_d.Text = jaguar.navigation.K_d.ToString();

        }

        private void ParamEdit_Load(object sender, EventArgs e)
        {

        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void label3_Click(object sender, EventArgs e)
        {

        }


    }
}
