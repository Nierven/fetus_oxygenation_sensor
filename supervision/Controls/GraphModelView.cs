using System.Collections.ObjectModel;
using System.Linq;
using System.Windows.Media;
using SciChart.Charting.Model.DataSeries;
using SciChart.Charting.Visuals.PointMarkers;
using SciChart.Charting.Visuals.RenderableSeries;

namespace Controls
{
    public sealed class GraphModelView
    {
        #region Properties

        private static readonly Color[] Colors =
        {
            Color.FromRgb(0, 114, 189),
            Color.FromRgb(217, 83, 25),
            Color.FromRgb(236, 176, 32),
            Color.FromRgb(126, 47, 141),
            Color.FromRgb(119, 171, 48),
            Color.FromRgb(77, 189, 237),
            Color.FromRgb(161, 20, 47)
        };

        private readonly ObservableCollection<IXyDataSeries<double, double>> _dataSeries = new ObservableCollection<IXyDataSeries<double, double>>();
        public ObservableCollection<IRenderableSeries> RenderableSeries { get; } = new ObservableCollection<IRenderableSeries>();

        /// <summary>Viewport used to manage user actions and auto scroll.</summary>
        public ScrollingViewportManager ViewportManager { get; } = new ScrollingViewportManager();

        #endregion

        public void AddPointDataSeries(string name = "Unknown")
        {
            // Create new Data series of type X=double, Y=double
            var series = new XyDataSeries<double, double>
            {
                FifoCapacity = 1000,
                SeriesName = name
            };

            // Set the data series on the chart's RenderableSeries
            RenderableSeries.Add(new XyScatterRenderableSeries
            {
                DataSeries = series,
                Stroke = Color.FromRgb(0, 0, 255),
                AntiAliasing = true,
                StrokeThickness = 5,
                PointMarker = new XPointMarker {StrokeThickness = 5}
            });

            _dataSeries.Add(series);
        }

        public void AddDataSeries(string name = "Unknown", double[] xArray = null, double[] yArray = null)
        {
            // Create new Data series of type X=double, Y=double
            var series = new XyDataSeries<double, double>
            {
                FifoCapacity = 2000,
                SeriesName = name
            };

            if (xArray != null && yArray != null)
                series.Append(xArray, yArray);

            // Gets the auto color to apply
            Color autoColor = System.Windows.Media.Colors.WhiteSmoke;
            foreach (Color color in Colors)
            {
                // Apply color only if all taken colors aren't equals
                if (RenderableSeries.All(renderableSeries => renderableSeries.Stroke != color))
                {
                    autoColor = color;
                    break;
                }
            }

            // Set the data series on the chart's RenderableSeries
            RenderableSeries.Add(new FastLineRenderableSeries
            {
                DataSeries = series,
                Stroke = autoColor,
                StrokeThickness = 2,
                AntiAliasing = true
            });

            _dataSeries.Add(series);
        }
        
        public void Update(string name, double t, double y)
        {
            var dataSeries = _dataSeries.ToList().Find(ds => ds.SeriesName == name);

            // Suspending updates is optional
            using (dataSeries.SuspendUpdates())
                dataSeries.Append(t, y);
        }
    }
}
