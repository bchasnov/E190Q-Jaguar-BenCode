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
                jaguar.navigation.Kpho = Double.Parse(txt.Text);
            }
        }


    }
}
