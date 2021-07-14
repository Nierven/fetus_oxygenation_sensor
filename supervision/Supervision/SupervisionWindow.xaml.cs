using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using Communications;
using supervision.Annotations;
// ReSharper disable InconsistentNaming

namespace Supervision
{
    /// <summary>
    /// Interaction logic for SupervisionWindow.xaml
    /// </summary>
    public partial class SupervisionWindow : Window, INotifyPropertyChanged
    {
        private double _spo2 = 0;

        public double SpO2
        {
            get => _spo2;
            set
            {
                _spo2 = value;
                OnPropertyChanged();
            }
        }

        private double _heartrate = 0;

        public double Heartrate
        {
            get => _heartrate;
            set
            {
                _heartrate = value;
                OnPropertyChanged();
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null) 
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        
        private void FilterButton_LP_5Hz_OnClick(object sender, RoutedEventArgs e)
        {
            if (Stream.UsedStream is AdvancedSerialPort sp)
                sp.WriteAsync(new[] {(byte) '0'});
        }

        private void FilterButton_LP_15Hz_OnClick(object sender, RoutedEventArgs e)
        {
            if (Stream.UsedStream is AdvancedSerialPort sp)
                sp.WriteAsync(new[] {(byte) '1'});
        }

        private void FilterButton_BP_3Hz_OnClick(object sender, RoutedEventArgs e)
        {
            if (Stream.UsedStream is AdvancedSerialPort sp)
                sp.WriteAsync(new[] {(byte) '4'});
        }

        private void FilterButton_BP_5Hz_OnClick(object sender, RoutedEventArgs e)
        {
            if (Stream.UsedStream is AdvancedSerialPort sp)
                sp.WriteAsync(new[] {(byte) '2'});
        }

        private void FilterButton_BP_15Hz_OnClick(object sender, RoutedEventArgs e)
        {
            if (Stream.UsedStream is AdvancedSerialPort sp)
                sp.WriteAsync(new[] {(byte) '3'});
        }

        private void StartButton_OnClick(object sender, RoutedEventArgs e)
        {
            StartButton.IsEnabled = false;
            StopButton.IsEnabled = true;
            if (Stream.UsedStream is AdvancedSerialPort sp)
                sp.WriteAsync(new[] {(byte) 's'});
        }

        private void StopButton_OnClick(object sender, RoutedEventArgs e)
        {
            StartButton.IsEnabled = true;
            StopButton.IsEnabled = false;
            if (Stream.UsedStream is AdvancedSerialPort sp)
                sp.WriteAsync(new[] {(byte) 't'});
        }
    }
}
