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

        float speed_sensitivity = 0.01f;

        Queue<float> LastHipCenterZs = new Queue<float>(new[] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f });
        Queue<float> LastSpineZs = new Queue<float>(new[] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f });
        Queue<float> LastHeadZs = new Queue<float>(new[] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f });
        Queue<float> LastKneeLeftZs = new Queue<float>(new[] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f });
        Queue<float> LastKneeRightZs = new Queue<float>(new[] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f });

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
                        green_if_same_z(body.Joints[JointType.HipLeft], body.Joints[JointType.HipRight]);
                        green_if_same_z(body.Joints[JointType.ShoulderLeft], body.Joints[JointType.ShoulderRight]);

                        // Wrists turn green if staying of frontal
                        green_if_same_z(body.Joints[JointType.WristLeft], body.Joints[JointType.WristRight]);

                        // Visual feedback on speed direction of hip center, spine, head and knees
                        plot_speed(body.Joints[JointType.HipCenter], LastHipCenterZs, speed_sensitivity);
                        plot_speed(body.Joints[JointType.Spine], LastSpineZs, speed_sensitivity);
                        plot_speed(body.Joints[JointType.Head], LastHeadZs, speed_sensitivity);
                        plot_speed(body.Joints[JointType.KneeLeft], LastKneeLeftZs, speed_sensitivity);
                        plot_speed(body.Joints[JointType.KneeRight], LastKneeRightZs, speed_sensitivity);

                        // Left and Right wrist coordinate printing
                        var rightWrist = body.Joints[JointType.WristRight];
                        XValueRight.Text = (100 * rightWrist.Position.X).ToString("0");
                        YValueRight.Text = (100 * rightWrist.Position.Y).ToString("0");
                        ZValueRight.Text = (100 * rightWrist.Position.Z).ToString("0");

                        var leftWrist = body.Joints[JointType.WristLeft];
                        XValueLeft.Text = (100 * leftWrist.Position.X).ToString("0");
                        YValueLeft.Text = (100 * leftWrist.Position.Y).ToString("0");
                        ZValueLeft.Text = (100 * leftWrist.Position.Z).ToString("0");
                       
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

        public void plot_speed(Joint joint, Queue<float> LastZPositions, float sensitivity)
        {
            float CurrentZPosition = joint.Position.Z;
            LastZPositions.Enqueue(CurrentZPosition);
            float LastZPosition = LastZPositions.Dequeue();

            if (CurrentZPosition - LastZPosition > sensitivity)
            {
                plot_joint(joint, Brushes.DarkBlue);
            }

            if (CurrentZPosition - LastZPosition < -sensitivity)
            {
                plot_joint(joint, Brushes.Red);
            }
        }

        public void green_if_same_z(Joint leftJoint, Joint rightJoint)
        {
            if (Math.Abs(leftJoint.Position.Z - rightJoint.Position.Z) <= 0.02)
            {
                plot_joint(leftJoint, Brushes.Green);
                plot_joint(rightJoint, Brushes.Green);
            }
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
