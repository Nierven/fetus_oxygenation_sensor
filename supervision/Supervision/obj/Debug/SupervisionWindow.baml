   M S B A M L   `   `   `  ����F  BSupervision, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null?$clr-namespace:Supervision.PropertiesSupervision.Properties  5   	   C ?Controls, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null5(clr-namespace:Controls;assembly=ControlsControls 5   	   #    Supervision.SupervisionWindow   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   @d2http://schemas.microsoft.com/expression/blend/2008     5   	   Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     5   	   ;
properties$clr-namespace:Supervision.Properties     5   	   =controls(clr-namespace:Controls;assembly=Controls     5   	   $	��450��5
   	   $	��600��6     � Title$#  Foetus oxymeter - Supervision��5   	    � DataContext 5   	   ��  �� RelativeSource �� *Self+ � WindowState 5   	   �� *WindowState+%  Supervision.Properties.Settings   Default �� Source# Z  �� Mode$ TwoWay=� � Left 5   	   �� *Left+# Z $ TwoWay=�
 � Top 5   	   �� *Top+# Z $ TwoWay=���5   	   �� *Width+# Z $ TwoWay=���5   	   �� *Height+# Z $ TwoWay=�.��5      � 	 � RowDefinitions	 5   
   �� 5      �� 5      $
��Auto �6   .+�5   
   � $��0��6   
 � ColumnDefinitions
 5      �� 5      $��3* �6#   $	��450��6.   �� 5      $
��Auto �6#   �� 5      $��1* �6#   $	��150��6.   .+�5#        Controls.Graph  $��0��6      Model 6-   �� *Model+�� 5&      $��1��6   $��Stretch=�6+   $��Center=�6G   � 5'      $��2��6   	 5(      �� 5)      $
��Auto �6$   �� 5*      $
��Auto �6$   �� 5+      $��1* �6$   �� 5,      $
��Auto �6$   �� 5-      $
��Auto �6$   .+�50      �� $��0��6   $
��10,5q�6)   $
��Bold"�67   $��24(�6I   SpO2: �� 6^   �6b   �� *SpO2+ %6}   �� 51      $��1��6   $��	10,0,10,5q�6)   $��20(�6<   Heart rate: �� 6W   �6[   �� *	Heartrate+ bmp6}   �� 53      $��3��6   $��10q�6*    �� Orientation$ Vertical=�66   .+�54      	� -   5         	� 	GroupName$ Filter��54   "   $��Low Pass - 5Hz��65   6N   	� 55      -   5        $ Filter��55   "   $��Low Pass - 15Hz��65   6O   	� 56      -   5        $ Filter��56   "   $��Band Pass - 3.3Hz��65   6Q   	� 57      -   5        $ Filter��57   "   $��Band Pass - 5Hz��65    d� 	IsChecked$
 TrueY�6O   6`   	� 58      -   5        $ Filter��58   "   $��Band Pass - 15Hz��65   6P   59      � 5;      $��4��6   $��10q�6$   
 5<      �� 5=      $��1* �6+   �� 5>      $��1* �6+   .+�5A      �� -   5        $��StartButton��5A      $��0��60   $��Start��6@   $��0,0,2,0q�6P   {�.  6s   �� 5B      -   5        $��
StopButton��5B      $��1��6/   $
��Stop��6?   $��0q�6N   6Y   5C      5D      5E      �� 5G   
   $��1��6    � 	Resources 5H       �� BasedOn �� 
TargetType �� Property �� Value��%\   '��       7���� 5I      8   " ��6J   .�5J      ��  � ��$	 2,0q�6/   5K      5L      .@�5N      �� .��5O        Controls.StatusIcon     Status$ OK=�6&      Size$ 13Q�62   5P      �� 5R      .��5S      �� -   5        $��StatusTextBlock��5S      $��Supervision started��63   5T      5U      5V      5W      