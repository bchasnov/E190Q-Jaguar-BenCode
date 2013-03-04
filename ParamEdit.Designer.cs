namespace DrRobot.JaguarControl
{
    partial class ParamEdit
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.contextMenuStrip1 = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.heToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.label1 = new System.Windows.Forms.Label();
            this.K_p = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.K_i = new System.Windows.Forms.TextBox();
            this.K_d = new System.Windows.Forms.TextBox();
            this.Kpho = new System.Windows.Forms.TextBox();
            this.Kalpha = new System.Windows.Forms.TextBox();
            this.Kbeta = new System.Windows.Forms.TextBox();
            this.trajThresh = new System.Windows.Forms.TextBox();
            this.msR = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.msL = new System.Windows.Forms.TextBox();
            this.textBox2 = new System.Windows.Forms.TextBox();
            this.textBox3 = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.checkBox1 = new System.Windows.Forms.CheckBox();
            this.contextMenuStrip1.SuspendLayout();
            this.SuspendLayout();
            // 
            // contextMenuStrip1
            // 
            this.contextMenuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.heToolStripMenuItem});
            this.contextMenuStrip1.Name = "contextMenuStrip1";
            this.contextMenuStrip1.Size = new System.Drawing.Size(88, 26);
            // 
            // heToolStripMenuItem
            // 
            this.heToolStripMenuItem.Name = "heToolStripMenuItem";
            this.heToolStripMenuItem.Size = new System.Drawing.Size(87, 22);
            this.heToolStripMenuItem.Text = "he";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(39, 20);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(26, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "K_p";
            this.label1.Click += new System.EventHandler(this.label1_Click);
            // 
            // K_p
            // 
            this.K_p.Location = new System.Drawing.Point(72, 13);
            this.K_p.Name = "K_p";
            this.K_p.Size = new System.Drawing.Size(76, 20);
            this.K_p.TabIndex = 2;
            this.K_p.TextChanged += new System.EventHandler(this.textBox1_TextChanged);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(71, 361);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 3;
            this.button1.Text = "Load";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(42, 46);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(22, 13);
            this.label2.TabIndex = 4;
            this.label2.Text = "K_i";
            // 
            // K_i
            // 
            this.K_i.Location = new System.Drawing.Point(72, 39);
            this.K_i.Name = "K_i";
            this.K_i.Size = new System.Drawing.Size(76, 20);
            this.K_i.TabIndex = 5;
            this.K_i.TextChanged += new System.EventHandler(this.K_i_TextChanged);
            // 
            // K_d
            // 
            this.K_d.Location = new System.Drawing.Point(71, 65);
            this.K_d.Name = "K_d";
            this.K_d.Size = new System.Drawing.Size(76, 20);
            this.K_d.TabIndex = 6;
            this.K_d.TextChanged += new System.EventHandler(this.K_d_TextChanged);
            // 
            // Kpho
            // 
            this.Kpho.Location = new System.Drawing.Point(72, 91);
            this.Kpho.Name = "Kpho";
            this.Kpho.Size = new System.Drawing.Size(76, 20);
            this.Kpho.TabIndex = 7;
            this.Kpho.TextChanged += new System.EventHandler(this.Kpho_TextChanged);
            // 
            // Kalpha
            // 
            this.Kalpha.Location = new System.Drawing.Point(72, 117);
            this.Kalpha.Name = "Kalpha";
            this.Kalpha.Size = new System.Drawing.Size(76, 20);
            this.Kalpha.TabIndex = 8;
            this.Kalpha.TextChanged += new System.EventHandler(this.Kalpha_TextChanged);
            // 
            // Kbeta
            // 
            this.Kbeta.Location = new System.Drawing.Point(72, 143);
            this.Kbeta.Name = "Kbeta";
            this.Kbeta.Size = new System.Drawing.Size(76, 20);
            this.Kbeta.TabIndex = 9;
            this.Kbeta.TextChanged += new System.EventHandler(this.Kbeta_TextChanged);
            // 
            // trajThresh
            // 
            this.trajThresh.Location = new System.Drawing.Point(72, 169);
            this.trajThresh.Name = "trajThresh";
            this.trajThresh.Size = new System.Drawing.Size(76, 20);
            this.trajThresh.TabIndex = 10;
            this.trajThresh.TextChanged += new System.EventHandler(this.trajThresh_TextChanged);
            // 
            // msR
            // 
            this.msR.Location = new System.Drawing.Point(72, 195);
            this.msR.Name = "msR";
            this.msR.Size = new System.Drawing.Size(76, 20);
            this.msR.TabIndex = 11;
            this.msR.TextChanged += new System.EventHandler(this.textBox7_TextChanged);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(42, 68);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(26, 13);
            this.label3.TabIndex = 12;
            this.label3.Text = "K_d";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(39, 94);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(32, 13);
            this.label4.TabIndex = 13;
            this.label4.Text = "Kpho";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(31, 120);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(40, 13);
            this.label5.TabIndex = 14;
            this.label5.Text = "Kalpha";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(31, 146);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(35, 13);
            this.label7.TabIndex = 15;
            this.label7.Text = "Kbeta";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(17, 172);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(54, 13);
            this.label6.TabIndex = 16;
            this.label6.Text = "trajThresh";
            // 
            // msL
            // 
            this.msL.Location = new System.Drawing.Point(72, 221);
            this.msL.Name = "msL";
            this.msL.Size = new System.Drawing.Size(76, 20);
            this.msL.TabIndex = 17;
            this.msL.TextChanged += new System.EventHandler(this.msL_TextChanged);
            // 
            // textBox2
            // 
            this.textBox2.Location = new System.Drawing.Point(70, 247);
            this.textBox2.Name = "textBox2";
            this.textBox2.Size = new System.Drawing.Size(76, 20);
            this.textBox2.TabIndex = 18;
            // 
            // textBox3
            // 
            this.textBox3.Location = new System.Drawing.Point(72, 273);
            this.textBox3.Name = "textBox3";
            this.textBox3.Size = new System.Drawing.Size(76, 20);
            this.textBox3.TabIndex = 19;
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(-18, 198);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(89, 13);
            this.label8.TabIndex = 20;
            this.label8.Text = "desiredRotRateR";
            this.label8.Click += new System.EventHandler(this.label8_Click);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(-16, 224);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(87, 13);
            this.label9.TabIndex = 21;
            this.label9.Text = "desiredRotRateL";
            // 
            // checkBox1
            // 
            this.checkBox1.AutoSize = true;
            this.checkBox1.Location = new System.Drawing.Point(155, 197);
            this.checkBox1.Name = "checkBox1";
            this.checkBox1.Size = new System.Drawing.Size(64, 17);
            this.checkBox1.TabIndex = 22;
            this.checkBox1.Text = "override";
            this.checkBox1.UseVisualStyleBackColor = true;
            this.checkBox1.CheckedChanged += new System.EventHandler(this.checkBox1_CheckedChanged);
            // 
            // ParamEdit
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(298, 396);
            this.Controls.Add(this.checkBox1);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.textBox3);
            this.Controls.Add(this.textBox2);
            this.Controls.Add(this.msL);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.msR);
            this.Controls.Add(this.trajThresh);
            this.Controls.Add(this.Kbeta);
            this.Controls.Add(this.Kalpha);
            this.Controls.Add(this.Kpho);
            this.Controls.Add(this.K_d);
            this.Controls.Add(this.K_i);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.K_p);
            this.Controls.Add(this.label1);
            this.Name = "ParamEdit";
            this.Text = "ParamEdit";
            this.Load += new System.EventHandler(this.ParamEdit_Load);
            this.contextMenuStrip1.ResumeLayout(false);
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.ContextMenuStrip contextMenuStrip1;
        private System.Windows.Forms.ToolStripMenuItem heToolStripMenuItem;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox K_p;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox K_i;
        private System.Windows.Forms.TextBox K_d;
        private System.Windows.Forms.TextBox Kpho;
        private System.Windows.Forms.TextBox Kalpha;
        private System.Windows.Forms.TextBox Kbeta;
        private System.Windows.Forms.TextBox trajThresh;
        private System.Windows.Forms.TextBox msR;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox msL;
        private System.Windows.Forms.TextBox textBox2;
        private System.Windows.Forms.TextBox textBox3;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.CheckBox checkBox1;


    }
}