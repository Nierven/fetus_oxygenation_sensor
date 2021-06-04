using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Input;
using supervision.Annotations;

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
    }
}
