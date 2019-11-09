using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Globalization;
using System.Runtime.Caching;

namespace KinectCoordinateMapping
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        CameraMode _mode = CameraMode.Color;

        KinectSensor _sensor;
        Skeleton[] _bodies = new Skeleton[6];

        Queue<float> LastHipCenterZs = new Queue<float>(new[] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f });

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            _sensor = KinectSensor.KinectSensors.Where(s => s.Status == KinectStatus.Connected).FirstOrDefault();

            if (_sensor != null)
            {
                _sensor.ColorStream.Enable();
                _sensor.DepthStream.Enable();
                _sensor.SkeletonStream.Enable();

                _sensor.AllFramesReady += Sensor_AllFramesReady;

                _sensor.Start();
            }
        }

        void Sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            // Color
            using (var frame = e.OpenColorImageFrame())
            {
                if (frame != null)
                {
                    if (_mode == CameraMode.Color)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Depth
            using (var frame = e.OpenDepthImageFrame())
            {
                if (frame != null)
                {
                    if (_mode == CameraMode.Depth)
                    {
                        camera.Source = frame.ToBitmap();
                    }
                }
            }

            // Body
            using (var frame = e.OpenSkeletonFrame())
            {
                if (frame != null)
                {
                    canvas.Children.Clear();

                    frame.CopySkeletonDataTo(_bodies);

                    var body = (from s in _bodies
                                where s.TrackingState == SkeletonTrackingState.Tracked
                                select s).FirstOrDefault();
                    if (body != null)
                    {
                        // COORDINATE MAPPING
                        plot_joint(body.Joints[JointType.HipCenter], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.Spine], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.ShoulderCenter], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.Head], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.ShoulderLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.ElbowLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.WristLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.HandLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.ShoulderRight], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.ElbowRight], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.WristRight], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.HandRight], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.HipLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.KneeLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.AnkleLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.FootLeft], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.HipRight], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.KneeRight], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.AnkleRight], Brushes.LightBlue);
                        plot_joint(body.Joints[JointType.FootRight], Brushes.LightBlue);


                        // Shoulders and hips turn green if the person is facing the camera
                        if (Math.Abs(body.Joints[JointType.HipLeft].Position.Z - body.Joints[JointType.HipRight].Position.Z) <= 0.02)
                        {
                            plot_joint(body.Joints[JointType.HipLeft], Brushes.Green);
                            plot_joint(body.Joints[JointType.HipRight], Brushes.Green);
                        }


                        if (Math.Abs(body.Joints[JointType.ShoulderLeft].Position.Z - body.Joints[JointType.ShoulderRight].Position.Z) <= 0.02)
                        {
                            plot_joint(body.Joints[JointType.ShoulderLeft], Brushes.Green);
                            plot_joint(body.Joints[JointType.ShoulderRight], Brushes.Green);
                        }

                        // Update hip position history to compute speed sign
                        float CurrentHipCenterZ = body.Joints[JointType.HipCenter].Position.Z;
                        LastHipCenterZs.Enqueue(CurrentHipCenterZ);
                        float LastHipCenterZ = LastHipCenterZs.Dequeue();

                        if (CurrentHipCenterZ - LastHipCenterZ > 0.01)
                        {
                            plot_joint(body.Joints[JointType.HipCenter], Brushes.DarkBlue);
                        }

                        if (CurrentHipCenterZ - LastHipCenterZ < -0.01)
                        {
                            plot_joint(body.Joints[JointType.HipCenter], Brushes.Red);
                        }


                        // Left and Right hand coordinate printing
                        var rightHand = body.Joints[JointType.WristRight];
                        XValueRight.Text = (100 * rightHand.Position.X).ToString("0");
                        YValueRight.Text = (100 * rightHand.Position.Y).ToString("0");
                        ZValueRight.Text = (100 * rightHand.Position.Z).ToString("0");

                        var leftHand = body.Joints[JointType.WristLeft];
                        XValueLeft.Text = (100 * leftHand.Position.X).ToString("0");
                        YValueLeft.Text = (100 * leftHand.Position.Y).ToString("0");
                        ZValueLeft.Text = (100 * leftHand.Position.Z).ToString("0");
                       
                    }
                }
            }
        }

        public void plot_joint(Joint joint, Brush color)
        {
            // 3D coordinates in meters
            SkeletonPoint skeletonPoint = joint.Position;

            // 2D coordinates in pixels
            Point point = new Point();

            if (_mode == CameraMode.Color)
            {
                // Skeleton-to-Color mapping
                ColorImagePoint colorPoint = _sensor.CoordinateMapper.MapSkeletonPointToColorPoint(skeletonPoint, ColorImageFormat.RgbResolution640x480Fps30);

                point.X = colorPoint.X;
                point.Y = colorPoint.Y;
            }
            else if (_mode == CameraMode.Depth) // Remember to change the Image and Canvas size to 320x240.
            {
                // Skeleton-to-Depth mapping
                DepthImagePoint depthPoint = _sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skeletonPoint, DepthImageFormat.Resolution320x240Fps30);

                point.X = depthPoint.X;
                point.Y = depthPoint.Y;
            }

            // DRAWING...
            Ellipse ellipse = new Ellipse
            {
                Fill = color,
                Width = 20,
                Height = 20
            };

            Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

            canvas.Children.Add(ellipse);
        }

        private void Window_Unloaded(object sender, RoutedEventArgs e)
        {
            if (_sensor != null)
            {
                _sensor.Stop();
            }
        }
    }

    enum CameraMode
    {
        Color,
        Depth
    }
}
