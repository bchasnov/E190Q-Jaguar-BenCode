namespace DrRobot.JaguarControl
{
    partial class Map
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
            this.trackBar1 = new System.Windows.Forms.TrackBar();
            this.button1 = new System.Windows.Forms.Button();
            this.mapScale = new System.Windows.Forms.Label();
            this.mapString = new System.Windows.Forms.TextBox();
            this.loadMap = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.trackBar1)).BeginInit();
            this.SuspendLayout();
            // 
            // trackBar1
            // 
            this.trackBar1.Location = new System.Drawing.Point(12, -1);
            this.trackBar1.Maximum = 100;
            this.trackBar1.Minimum = 1;
            this.trackBar1.Name = "trackBar1";
            this.trackBar1.Size = new System.Drawing.Size(264, 45);
            this.trackBar1.TabIndex = 0;
            this.trackBar1.TickFrequency = 10;
            this.trackBar1.TickStyle = System.Windows.Forms.TickStyle.None;
            this.trackBar1.Value = 1;
            this.trackBar1.Scroll += new System.EventHandler(this.trackBar1_Scroll);
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(334, 3);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 1;
            this.button1.Text = "Print Map";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // mapScale
            // 
            this.mapScale.AutoSize = true;
            this.mapScale.Location = new System.Drawing.Point(270, 8);
            this.mapScale.Name = "mapScale";
            this.mapScale.Size = new System.Drawing.Size(58, 13);
            this.mapScale.TabIndex = 2;
            this.mapScale.Text = "map_scale";
            // 
            // mapString
            // 
            this.mapString.Location = new System.Drawing.Point(12, 32);
            this.mapString.Name = "mapString";
            this.mapString.Size = new System.Drawing.Size(478, 20);
            this.mapString.TabIndex = 3;
            this.mapString.TextChanged += new System.EventHandler(this.mapString_TextChanged);
            // 
            // loadMap
            // 
            this.loadMap.Location = new System.Drawing.Point(415, 3);
            this.loadMap.Name = "loadMap";
            this.loadMap.Size = new System.Drawing.Size(75, 23);
            this.loadMap.TabIndex = 4;
            this.loadMap.Text = "loadMap";
            this.loadMap.UseVisualStyleBackColor = true;
            this.loadMap.Click += new System.EventHandler(this.loadMap_Click);
            // 
            // Map
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(936, 548);
            this.Controls.Add(this.loadMap);
            this.Controls.Add(this.mapString);
            this.Controls.Add(this.mapScale);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.trackBar1);
            this.Name = "Map";
            this.Text = "Map";
            this.Load += new System.EventHandler(this.Map_Load);
            ((System.ComponentModel.ISupportInitialize)(this.trackBar1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TrackBar trackBar1;
        private System.Windows.Forms.Button button1;
        private System.Windows.Forms.Label mapScale;
        private System.Windows.Forms.TextBox mapString;
        private System.Windows.Forms.Button loadMap;

    }
}