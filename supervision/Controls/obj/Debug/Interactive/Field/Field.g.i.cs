#pragma checksum "..\..\..\..\Interactive\Field\Field.xaml" "{8829d00f-11b8-4213-878b-770e8597ac16}" "681B33E3A87230B28C2D7498D3FF766A7298FB7DAF345697864144FC772363B3"
//------------------------------------------------------------------------------
// <auto-generated>
//     Ce code a été généré par un outil.
//     Version du runtime :4.0.30319.42000
//
//     Les modifications apportées à ce fichier peuvent provoquer un comportement incorrect et seront perdues si
//     le code est régénéré.
// </auto-generated>
//------------------------------------------------------------------------------

using Controls.Interactive.Field;
using HelixToolkit.Wpf.SharpDX;
using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;


namespace Controls.Interactive.Field {
    
    
    /// <summary>
    /// Field
    /// </summary>
    public partial class Field : System.Windows.Controls.UserControl, System.Windows.Markup.IComponentConnector {
        
        
        #line 23 "..\..\..\..\Interactive\Field\Field.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal HelixToolkit.Wpf.SharpDX.Viewport3DX ViewControl;
        
        #line default
        #line hidden
        
        
        #line 107 "..\..\..\..\Interactive\Field\Field.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.ComboBox RobotsComboBox;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/Controls;component/interactive/field/field.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\..\..\Interactive\Field\Field.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            
            #line 10 "..\..\..\..\Interactive\Field\Field.xaml"
            ((Controls.Interactive.Field.Field)(target)).Loaded += new System.Windows.RoutedEventHandler(this.Field_OnLoaded);
            
            #line default
            #line hidden
            
            #line 10 "..\..\..\..\Interactive\Field\Field.xaml"
            ((Controls.Interactive.Field.Field)(target)).Unloaded += new System.Windows.RoutedEventHandler(this.Field_OnUnloaded);
            
            #line default
            #line hidden
            return;
            case 2:
            
            #line 14 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Input.CommandBinding)(target)).Executed += new System.Windows.Input.ExecutedRoutedEventHandler(this.AddObstacleCommand_OnExecuted);
            
            #line default
            #line hidden
            return;
            case 3:
            
            #line 15 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Input.CommandBinding)(target)).CanExecute += new System.Windows.Input.CanExecuteRoutedEventHandler(this.DeleteObstacleCommand_OnCanExecute);
            
            #line default
            #line hidden
            
            #line 15 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Input.CommandBinding)(target)).Executed += new System.Windows.Input.ExecutedRoutedEventHandler(this.DeleteObstacleCommand_OnExecuted);
            
            #line default
            #line hidden
            return;
            case 4:
            
            #line 16 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Input.CommandBinding)(target)).CanExecute += new System.Windows.Input.CanExecuteRoutedEventHandler(this.DeleteAllCommand_OnCanExecute);
            
            #line default
            #line hidden
            
            #line 16 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Input.CommandBinding)(target)).Executed += new System.Windows.Input.ExecutedRoutedEventHandler(this.DeleteAllCommand_OnExecuted);
            
            #line default
            #line hidden
            return;
            case 5:
            
            #line 17 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Input.CommandBinding)(target)).CanExecute += new System.Windows.Input.CanExecuteRoutedEventHandler(this.ResetViewCommand_OnCanExecute);
            
            #line default
            #line hidden
            
            #line 17 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Input.CommandBinding)(target)).Executed += new System.Windows.Input.ExecutedRoutedEventHandler(this.ResetViewCommand_OnExecuted);
            
            #line default
            #line hidden
            return;
            case 6:
            this.ViewControl = ((HelixToolkit.Wpf.SharpDX.Viewport3DX)(target));
            
            #line 46 "..\..\..\..\Interactive\Field\Field.xaml"
            this.ViewControl.PreviewMouseDoubleClick += new System.Windows.Input.MouseButtonEventHandler(this.ViewControl_OnPreviewMouseDoubleClick);
            
            #line default
            #line hidden
            return;
            case 7:
            this.RobotsComboBox = ((System.Windows.Controls.ComboBox)(target));
            return;
            case 8:
            
            #line 112 "..\..\..\..\Interactive\Field\Field.xaml"
            ((System.Windows.Controls.Button)(target)).Click += new System.Windows.RoutedEventHandler(this.RobotSelectionClearButton_OnClick);
            
            #line default
            #line hidden
            return;
            }
            this._contentLoaded = true;
        }
    }
}

