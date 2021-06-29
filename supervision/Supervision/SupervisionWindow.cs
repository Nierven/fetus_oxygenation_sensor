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

            Filtrage.FiltrePasseBasButterworthOrdre2Init(_redAcFilter, 600, 50);
            Filtrage.FiltrePasseBasButterworthOrdre2Init(_redDcFilter, 600, 50);
            Filtrage.FiltrePasseBasButterworthOrdre2Init(_irAcFilter, 600, 50);
            Filtrage.FiltrePasseBasButterworthOrdre2Init(_irDcFilter, 600, 50);

            Model.AddDataSeries("Red - AC");
            Model.AddDataSeries("Red - DC");

            Model.AddDataSeries("IR - AC");
            Model.AddDataSeries("IR - DC");

            Stream.MessageDecoded += Stream_MessageDecoded;
            StatusTextBlock.Text = Stream.IsOpen.ToString();
        }

        private readonly FiltreOrdre2 _redAcFilter = new FiltreOrdre2();
        private readonly FiltreOrdre2 _redDcFilter = new FiltreOrdre2();
        private readonly FiltreOrdre2 _irAcFilter = new FiltreOrdre2();
        private readonly FiltreOrdre2 _irDcFilter = new FiltreOrdre2();

        private void Stream_MessageDecoded(object sender, MessageDecodedEventArgs e)
        {
            switch (e.MessageDecoded.Command)
            {
                case Commands.Unknown:
                    double red_ac = (e.MessageDecoded.Payload[0] << 8) + e.MessageDecoded.Payload[1];
                    double red_dc = (e.MessageDecoded.Payload[2] << 8) + e.MessageDecoded.Payload[3];
                    double ir_ac = (e.MessageDecoded.Payload[4] << 8) + e.MessageDecoded.Payload[5];
                    double ir_dc = (e.MessageDecoded.Payload[6] << 8) + e.MessageDecoded.Payload[7];

                    red_ac *= 3.3 / 16384;
                    red_dc *= 3.3 / 16384;
                    ir_ac *= 3.3 / 16384;
                    ir_dc *= 3.3 / 16384;
                    
                    red_ac = Math.Max(0, Math.Min(3.3, red_ac));
                    red_dc = Math.Max(0, Math.Min(3.3, red_dc));
                    ir_ac = Math.Max(0, Math.Min(3.3, ir_ac));
                    ir_dc = Math.Max(0, Math.Min(3.3, ir_dc));

                    red_ac = Filtrage.FiltreOrdre2TpsReel(_redAcFilter, red_ac);
                    red_dc = Filtrage.FiltreOrdre2TpsReel(_redDcFilter, red_dc);
                    ir_ac = Filtrage.FiltreOrdre2TpsReel(_irAcFilter, ir_ac);
                    ir_dc = Filtrage.FiltreOrdre2TpsReel(_irDcFilter, ir_dc);

                    Model.Update("Red - AC", T, red_ac);
                    Model.Update("Red - DC", T, red_dc);
                    Model.Update("IR - AC", T, ir_ac);
                    Model.Update("IR - DC", T, ir_dc);
                    break;
                case Commands.Test:
                    SpO2 = (e.MessageDecoded.Payload[0] << 8) + e.MessageDecoded.Payload[1];
                    Heartrate = (e.MessageDecoded.Payload[2] << 8) + e.MessageDecoded.Payload[3];
                    break;
            }
        }

        private readonly DateTime _t0 = DateTime.Now;
        private double T => (DateTime.Now - _t0).TotalSeconds;

        public MessageStream Stream { get; } = new MessageStream(new AdvancedSerialPort("COM7", 115200));

        public GraphModelView Model { get; } = new GraphModelView();
    }
}
