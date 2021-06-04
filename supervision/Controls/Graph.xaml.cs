using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Controls;
using Controls.Annotations;
using Controls.Properties;

namespace Controls
{
    public sealed partial class Graph : UserControl, INotifyPropertyChanged
    {
        #region Properties

        /// <summary>Model to display in the graph.</summary>
        public static readonly DependencyProperty ModelProperty =
            DependencyProperty.Register(nameof(Model), typeof(GraphModelView), typeof(Graph), new PropertyMetadata(new GraphModelView(), Model_PropertyChangedCallback));

        private static void Model_PropertyChangedCallback(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is Graph widget)
                widget.Model = (GraphModelView)e.NewValue;
        }

        /// <summary>Model to display in the graph.</summary>
        public GraphModelView Model
        {
            get => (GraphModelView)GetValue(ModelProperty);
            set
            {
                SetValue(ModelProperty, value);
                OnPropertyChanged();
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
            => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));

        #endregion

        public Graph()
        {
            // Used to import SciChart.Drawing.DirectX
            AssemblyImporter.ImportAssemblies();

            InitializeComponent();
            // X & Y Axis auto scroll on the last possible period of time with this manager
            ChartSurface.ViewportManager = Model.ViewportManager;
        }
    }
}
