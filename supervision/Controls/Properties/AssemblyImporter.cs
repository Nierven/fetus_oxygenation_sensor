using System;

namespace Controls.Properties
{
    /// <summary>
    /// /!\ BEWARE - THIS CLASS IS USED TO IMPORT ASSEMBLIES MSBUILD LEAVES BEHIND BECAUSE IT DETERMINED IT ISN'T USED (WHICH IS FALSE IN OUR CASE)
    /// </summary>
    internal static class AssemblyImporter
    {
        private static readonly Type[] Types =
        {
            typeof(SciChart.Drawing.DirectX.Rendering.IDirectXRenderSurface),
            typeof(SciChart.Charting.DrawingTools.TradingAnnotations.BrushAnnotation)
        };

        internal static void ImportAssemblies()
        {
            foreach (Type type in Types)
            {
                // ReSharper disable once UnusedParameter.Local
                void Nop(Type _) { }
                Nop(type);
            }
        }
    }
}
