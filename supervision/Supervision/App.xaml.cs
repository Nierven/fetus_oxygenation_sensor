using System.Windows;
using SciChart.Charting.Visuals;

namespace Supervision
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        public App()
        {
            // ReSharper disable once StringLiteralTypo
            SciChartSurface.SetRuntimeLicenseKey(@"<LicenseContract>
                <Customer>University of Toulon</Customer>
                <OrderId>EDUCATIONAL-USE-0109</OrderId>
                <LicenseCount>1</LicenseCount>
                <IsTrialLicense>false</IsTrialLicense>
                <SupportExpires>11/04/2019 00:00:00</SupportExpires>
                <ProductCode>SC-WPF-SDK-PRO-SITE</ProductCode>
                <KeyCode>lwABAQEAAABZVzOfQ0zVAQEAewBDdXN0b21lcj1Vbml2ZXJzaXR5IG9mICBUb3Vsb247T3JkZXJJZD1FRFVDQVRJT05BTC1VU0UtMDEwOTtTdWJzY3JpcHRpb25WYWxpZFRvPTA0LU5vdi0yMDE5O1Byb2R1Y3RDb2RlPVNDLVdQRi1TREstUFJPLVNJVEWDf0QgB8GnCQXI6yAqNM2njjnGbUt2KsujTDzeE+k69K1XYVF1s1x1Hb/i/E3GHaU=</KeyCode> 
            </LicenseContract>");
        }

        private void App_OnExit(object sender, ExitEventArgs e)
        {
            Supervision.Properties.Settings.Default.Save();
        }
    }
}
