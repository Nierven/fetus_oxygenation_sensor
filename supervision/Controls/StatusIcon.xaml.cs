using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Controls.Annotations;

using Color = System.Drawing.Color;

namespace Controls
{
    public enum StatusInfo
    {
        None,
        OK,
        Info,
        Warning,
        Error,
        Red,
        Blue,
        Yellow,
        Orange
    }

    public sealed partial class StatusIcon : UserControl, INotifyPropertyChanged
    {
        public StatusIcon()
        {
            InitializeComponent();
            _blinkTimer.Tick += BlinkTimer_OnTick;
        }

        private readonly Dictionary<StatusInfo, Color> _colors = new Dictionary<StatusInfo, Color>
        {
            {StatusInfo.None, Color.DimGray},
            {StatusInfo.OK, Color.Green},
            {StatusInfo.Info, Color.Yellow},
            {StatusInfo.Warning, Color.Orange},
            {StatusInfo.Error, Color.Red},
            {StatusInfo.Red, Color.Red},
            {StatusInfo.Blue, Color.Blue},
            {StatusInfo.Yellow, Color.Yellow},
            {StatusInfo.Orange, Color.Orange},
        };

        private readonly DispatcherTimer _blinkTimer = new DispatcherTimer(DispatcherPriority.Render);

        private bool _isMainColor = true;
        private TimeSpan _blinkInterval;
        private TimeSpan _mainColorInterval;
        private void BlinkTimer_OnTick(object sender, EventArgs e)
        {
            // Changes from main color to static color and opposite
            SetIconColor(_isMainColor ? _colors[StatusInfo.None] : _colors[Status]);
            // Changes color status
            _isMainColor ^= true;

            // If main color, updates interval to a short time, else get the normal interval
            _blinkTimer.Interval = _isMainColor ? _mainColorInterval : _blinkInterval;
        }

        private void UpdateStatus()
        {
            switch (Status)
            {
                case StatusInfo.Info:
                    _mainColorInterval = TimeSpan.FromSeconds(0.35);
                    _blinkInterval = TimeSpan.FromSeconds(2);
                    _blinkTimer.Interval = _blinkInterval;
                    _blinkTimer.Start();
                    break;
                case StatusInfo.Warning:
                    _mainColorInterval = TimeSpan.FromSeconds(0.25);
                    _blinkInterval = TimeSpan.FromSeconds(1.5);
                    _blinkTimer.Interval = _blinkInterval;
                    _blinkTimer.Start();
                    break;
                case StatusInfo.Error:
                    _mainColorInterval = TimeSpan.FromSeconds(0.1);
                    _blinkInterval = TimeSpan.FromSeconds(1);
                    _blinkTimer.Interval = _blinkInterval;
                    _blinkTimer.Start();
                    break;
            }

            // Sets the appropriate color depending on the status
            SetIconColor(_colors[Status]);
        }

        private void SetIconColor(Color color)
            => Icon.Fill = new SolidColorBrush(System.Windows.Media.Color.FromArgb(color.A, color.R, color.G, color.B));

        #region Dependency properties

        /// <summary>Status to display on the icon.</summary>
        public static readonly DependencyProperty StatusProperty =
            DependencyProperty.Register(nameof(Status), typeof(StatusInfo), typeof(StatusIcon), new PropertyMetadata(StatusInfo.None, Status_PropertyChangedCallback));

        private static void Status_PropertyChangedCallback(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is StatusIcon widget)
                widget.Status = (StatusInfo)e.NewValue;
        }

        /// <summary>Status to display on the icon.</summary>
        public StatusInfo Status
        {
            get => (StatusInfo)GetValue(StatusProperty);
            set
            {
                SetValue(StatusProperty, value);
                UpdateStatus();
                OnPropertyChanged();
            }
        }

        /// <summary>Size of the icon.</summary>
        public static readonly DependencyProperty SizeProperty =
            DependencyProperty.Register(nameof(Size), typeof(double), typeof(StatusIcon), new PropertyMetadata(10d, Size_PropertyChangedCallback));

        private static void Size_PropertyChangedCallback(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is StatusIcon widget)
                widget.Size = (double)e.NewValue;
        }

        /// <summary>Size of the icon.</summary>
        public double Size
        {
            get => (double)GetValue(SizeProperty);
            set
            {
                SetValue(SizeProperty, value);
                OnPropertyChanged();
            }
        }

        /// <summary>Shape of the icon.</summary>
        public static readonly DependencyProperty RectangleShapeProperty =
            DependencyProperty.Register(nameof(RectangleShape), typeof(bool), typeof(StatusIcon), new PropertyMetadata(false, RectangleShape_PropertyChangedCallback));

        private static void RectangleShape_PropertyChangedCallback(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is StatusIcon widget)
                widget.RectangleShape = (bool)e.NewValue;
        }

        /// <summary>Shape of the icon.</summary>
        public bool RectangleShape
        {
            get => (bool)GetValue(RectangleShapeProperty);
            set
            {
                SetValue(RectangleShapeProperty, value);

                if (value)
                {
                    Icon.RadiusX = 0;
                    Icon.RadiusY = 0;
                }
                else
                {
                    Icon.SetBinding(Rectangle.RadiusXProperty, nameof(Size));
                    Icon.SetBinding(Rectangle.RadiusYProperty, nameof(Size));
                }

                OnPropertyChanged();
            }
        }

        #endregion

        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        private void OnPropertyChanged([CallerMemberName] string propertyName = null) 
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}
