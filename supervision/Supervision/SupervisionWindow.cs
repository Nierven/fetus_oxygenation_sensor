using Communications;
using Controls;
using System;

namespace Supervision
{
    public partial class SupervisionWindow
    {
        public SupervisionWindow()
        {
            InitializeComponent();

            Model.AddDataSeries("Red - Raw");
            Model.AddDataSeries("Red - AC");
            Model.AddDataSeries("Red - DC");
            
            Model.AddDataSeries("IR - Raw");
            Model.AddDataSeries("IR - AC");
            Model.AddDataSeries("IR - DC");

            Model.AddDataSeries("SpO2");
            Model.AddDataSeries("Heartrate");
            Model.RenderableSeries[6].IsVisible = false;
            Model.RenderableSeries[7].IsVisible = false;

            Stream.MessageDecoded += Stream_MessageDecoded;
            StatusTextBlock.Text = Stream.IsOpen.ToString();
        }

        private void Stream_MessageDecoded(object sender, MessageDecodedEventArgs e)
        {
            switch (e.MessageDecoded.Command)
            {
                case Commands.Unknown:
                    double red_raw = (e.MessageDecoded.Payload[0] << 8) + e.MessageDecoded.Payload[1];
                    double red_ac = (e.MessageDecoded.Payload[2] << 8) + e.MessageDecoded.Payload[3];
                    double red_dc = (e.MessageDecoded.Payload[4] << 8) + e.MessageDecoded.Payload[5];
                    double ir_raw = (e.MessageDecoded.Payload[6] << 8) + e.MessageDecoded.Payload[7];
                    double ir_ac = (e.MessageDecoded.Payload[8] << 8) + e.MessageDecoded.Payload[9];
                    double ir_dc = (e.MessageDecoded.Payload[10] << 8) + e.MessageDecoded.Payload[11];
                    
                    red_raw *= 3.3 / 16384;
                    red_ac *= 3.3 / 16384;
                    red_dc *= 3.3 / 16384;
                    ir_raw *= 3.3 / 16384;
                    ir_ac *= 3.3 / 16384;
                    ir_dc *= 3.3 / 16384;
                    
                    red_raw = Math.Max(0, Math.Min(3.3, red_raw));
                    red_ac = Math.Max(0, Math.Min(3.3, red_ac));
                    red_dc = Math.Max(0, Math.Min(3.3, red_dc));
                    ir_raw = Math.Max(0, Math.Min(3.3, ir_raw));
                    ir_ac = Math.Max(0, Math.Min(3.3, ir_ac));
                    ir_dc = Math.Max(0, Math.Min(3.3, ir_dc));
                    
                    Model.Update("Red - Raw", T, red_raw);
                    Model.Update("Red - AC", T, red_ac);
                    Model.Update("Red - DC", T, red_dc);
                    Model.Update("IR - Raw", T, ir_raw);
                    Model.Update("IR - AC", T, ir_ac);
                    Model.Update("IR - DC", T, ir_dc);
                    break;
                case Commands.Test:
                    //Model.Update("SpO2", T - 0.00001, SpO2);
                    //Model.Update("Heartrate", T - 0.00001, Heartrate);

                    SpO2 = (e.MessageDecoded.Payload[0] << 8) + e.MessageDecoded.Payload[1];
                    Heartrate = (e.MessageDecoded.Payload[2] << 8) + e.MessageDecoded.Payload[3];

                    //SpO2 *= 3.3 / 16384;
                    //Heartrate *= 3.3 / 16384;

                    //SpO2 = Math.Max(0, Math.Min(3.3, SpO2));
                    //Heartrate = Math.Max(0, Math.Min(3.3, Heartrate));

                    Model.Update("SpO2", T, SpO2);
                    Model.Update("Heartrate", T, Heartrate);
                    break;
            }
        }

        private readonly DateTime _t0 = DateTime.Now;
        private double T => (DateTime.Now - _t0).TotalSeconds;

        public MessageStream Stream { get; } = new MessageStream(new AdvancedSerialPort("COM4", 115200));

        public GraphModelView Model { get; } = new GraphModelView();
    }
}
