using SciChart.Charting.Services;
using SciChart.Charting.ViewportManagers;
using SciChart.Charting.Visuals;
using SciChart.Charting.Visuals.Axes;
using SciChart.Data.Model;

namespace Controls
{
    /// <summary>
    /// The following class will apply a scrolling window to the chart unless the user is zooming or panning.
    /// </summary>
    public class ScrollingViewportManager : DefaultViewportManager
    {
        private ISciChartSurface _parentSurface;
        public override void AttachSciChartSurface(ISciChartSurface scs)
        {
            base.AttachSciChartSurface(scs);
            _parentSurface = scs;
        }

        private IRange GetOptimalRange(IAxisParams axis)
        {
            // The Current Axis VisibleRange
            var currentVisibleRange = axis.VisibleRange.AsDoubleRange();
            if (_parentSurface.ZoomState == ZoomStates.UserZooming)
                // Don't scroll if user is zooming
                return currentVisibleRange;

            // The MaxRange is the VisibleRange on the Axis if we were to zoom to fit all data
            return (IRange)axis.GetMaximumRange().Clone();
        }

        protected override IRange OnCalculateNewXRange(IAxis xAxis)
            => GetOptimalRange(xAxis);

        protected override IRange OnCalculateNewYRange(IAxis yAxis, RenderPassInfo renderPassInfo)
            => GetOptimalRange(yAxis);
    }
}