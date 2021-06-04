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

            Model.AddDataSeries("Red - AC");
            Model.AddDataSeries("Red - DC");

            Model.AddDataSeries("IR - AC");
            Model.AddDataSeries("IR - DC");

            Stream.MessageDecoded += Stream_MessageDecoded;
            StatusTextBlock.Text = Stream.IsOpen.ToString();
        }

        private void Stream_MessageDecoded(object sender, MessageDecodedEventArgs e)
        {
            switch (e.MessageDecoded.Command)
            {
                case Commands.Unknown:
                    Model.Update("Red - AC", T, (e.MessageDecoded.Payload[0] << 8) + e.MessageDecoded.Payload[1]);
                    Model.Update("Red - DC", T, (e.MessageDecoded.Payload[2] << 8) + e.MessageDecoded.Payload[3]);
                    Model.Update("IR - AC", T, (e.MessageDecoded.Payload[4] << 8) + e.MessageDecoded.Payload[5]);
                    Model.Update("IR - DC", T, (e.MessageDecoded.Payload[6] << 8) + e.MessageDecoded.Payload[7]);
                    break;
                case Commands.Test:
                    SpO2 = (e.MessageDecoded.Payload[0] << 8) + e.MessageDecoded.Payload[1];
                    Heartrate = (e.MessageDecoded.Payload[2] << 8) + e.MessageDecoded.Payload[3];
                    break;
                default:
                    break;
            }
        }

        private readonly DateTime _t0 = DateTime.Now;
        private double T => (DateTime.Now - _t0).TotalSeconds;

        public MessageStream Stream { get; } = new MessageStream(new AdvancedSerialPort("COM7", 115200));

        public GraphModelView Model { get; } = new GraphModelView();
    }
}
