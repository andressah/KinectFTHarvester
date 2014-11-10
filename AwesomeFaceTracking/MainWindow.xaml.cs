using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Ventuz.OSC;

using Microsoft.Kinect;
using Microsoft.Kinect.Toolkit.FaceTracking;
using Microsoft.Win32;
using System.Threading;

namespace AwesomeFaceTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor kinectSensor;
        FaceTracker faceTracker;
        private byte[] colorPixelData;
        private short[] depthPixelData;
        private Skeleton[] skeletonData;
        private Boolean logToFile = false, writeToOSC = false;
        private volatile Boolean replayFile = false;
        private volatile int replaySpeed;
        private FeaturePoint[] featurePoints = (FeaturePoint[]) Enum.GetValues(typeof(FeaturePoint));
        UdpWriter oscWriter = new UdpWriter("127.0.0.1", Convert.ToInt32(9000));

        public MainWindow()
        {
            InitializeComponent();

            // For a KinectSensor to be detected, we can plug it in after the application has been started.
            KinectSensor.KinectSensors.StatusChanged += KinectSensors_StatusChanged;
            // Or it's already plugged in, so we will look for it.
            var kinect = KinectSensor.KinectSensors.FirstOrDefault(k => k.Status == KinectStatus.Connected);
            if (kinect != null)
            {
                OpenKinect(kinect);
            }
            replaySpeed = (int) slider_replay_speed.Value;

            start_btn.Click += new RoutedEventHandler(this.toggleLogging);
            osc_btn.Click += new RoutedEventHandler(this.toggleOSC);
            btn_replay.Click += new RoutedEventHandler(this.toggleReplay);
            btn_filechoose.Click += new RoutedEventHandler(this.filechoose);
    
        }

        private void filechoose(object sender, System.EventArgs e)
        {
            // Configure open file dialog box
            Microsoft.Win32.OpenFileDialog dlg = new Microsoft.Win32.OpenFileDialog();
            dlg.FileName = "Document"; // Default file name
            dlg.DefaultExt = ".txt"; // Default file extension
            dlg.Filter = "Text documents (.*)|*.*"; // Filter files by extension 

            // Show open file dialog box
            Nullable<bool> result = dlg.ShowDialog();

            // Process open file dialog box results 
            if (result == true)
            {
                // Open document 
                string filename = dlg.FileName;
                txt_replay.Text = filename;
            }
            
        }

        void toggleLogging(Object sender,
                           RoutedEventArgs e)
        {

            logToFile = !logToFile;
            if (logToFile)
            {
                start_btn.Content = "Stop";
            }
            else
            {
                start_btn.Content = "Start";
            }
        }

        void toggleOSC(Object sender,
                           RoutedEventArgs e)
        {

            writeToOSC = !writeToOSC;
            if (writeToOSC)
            {
                osc_btn.Content = "OSC Stop";
            }
            else
            {
                osc_btn.Content = "OSC Start";
            }
        }

        void toggleReplay(Object sender,
                              RoutedEventArgs e)
        {
            replayFile = !replayFile;
            if (replayFile)
            {
                btn_replay.Content = "Stop Replay";
                Thread replayThread = new Thread(replay);
                replayThread.Start(txt_replay.Text);
            }
            else
            {
                btn_replay.Content = "Start Replay";
            }
        }

        void replay(object data)
        {
           String file = (String) data;
           if (File.Exists(file))
            {
                while (replayFile)
                {
                    using (StreamReader f = new StreamReader(file))
                    {
                        //Remove Heading
                        f.ReadLine();
                        while (!f.EndOfStream)
                        {
                            Thread.Sleep(1000 / replaySpeed);
                            sendToOsc(f.ReadLine(), "kinect", oscWriter);
                        }

                    }
                }
            }
        }
        void sendToOsc(string line, String channel, UdpWriter oscWriter)
        {
            String[] splitted = line.Split(' ');
            int i = 0;
            //Remove the Label
            i++;
            OscBundle b = new OscBundle();
            b.AddElement(new OscElement("/" + channel + "/timestamp", splitted[i++]));
            foreach (FeaturePoint fp in featurePoints)
            {
                b.AddElement(new OscElement("/" + channel + "/" + fp.ToString() + "/x", splitted[i++]));
                b.AddElement(new OscElement("/" + channel + "/" + fp.ToString() + "/y", splitted[i++]));
                b.AddElement(new OscElement("/" + channel + "/" + fp.ToString() + "/z", splitted[i++]));
            }
            oscWriter.Send(b);

        }


        /// <summary>
        /// Handles the StatusChanged event of the KinectSensors control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="Microsoft.Kinect.StatusChangedEventArgs"/> instance containing the event data.</param>
        void KinectSensors_StatusChanged(object sender, StatusChangedEventArgs e)
        {
            if (e.Status == KinectStatus.Connected)
            {
                OpenKinect(e.Sensor);
            }
        }

        /// <summary>
        /// Opens the kinect.
        /// </summary>
        /// <param name="newSensor">The new sensor.</param>
        private void OpenKinect(KinectSensor newSensor)
        {
            kinectSensor = newSensor;

            // Initialize all the necessary streams:
            // - ColorStream with default format
            // - DepthStream with Near mode
            // - SkeletonStream with tracking in NearReange and Seated mode.

            kinectSensor.ColorStream.Enable();

            kinectSensor.DepthStream.Range = DepthRange.Near;
            kinectSensor.DepthStream.Enable(DepthImageFormat.Resolution80x60Fps30);

            kinectSensor.SkeletonStream.EnableTrackingInNearRange = true;
            kinectSensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
            kinectSensor.SkeletonStream.Enable(new TransformSmoothParameters() { Correction = 0.5f, JitterRadius = 0.05f, MaxDeviationRadius = 0.05f, Prediction = 0.5f, Smoothing = 0.5f });

            // Listen to the AllFramesReady event to receive KinectSensor's data.
            kinectSensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(kinectSensor_AllFramesReady);

            // Initialize data arrays
            colorPixelData = new byte[kinectSensor.ColorStream.FramePixelDataLength];
            depthPixelData = new short[kinectSensor.DepthStream.FramePixelDataLength];
            skeletonData = new Skeleton[6];

            // Starts the Sensor
            kinectSensor.Start();

            // Initialize a new FaceTracker with the KinectSensor
            faceTracker = new FaceTracker(kinectSensor);
        }

        /// <summary>
        /// Handles the AllFramesReady event of the kinectSensor control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="Microsoft.Kinect.AllFramesReadyEventArgs"/> instance containing the event data.</param>
        void kinectSensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            // Retrieve each single frame and copy the data
            using (ColorImageFrame colorImageFrame = e.OpenColorImageFrame())
            {
                if (colorImageFrame == null)
                    return;
                colorImageFrame.CopyPixelDataTo(colorPixelData);
            }

            using (DepthImageFrame depthImageFrame = e.OpenDepthImageFrame())
            {
                if (depthImageFrame == null)
                    return;
                depthImageFrame.CopyPixelDataTo(depthPixelData);
            }

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame == null)
                    return;
                skeletonFrame.CopySkeletonDataTo(skeletonData);
            }

            // Retrieve the first tracked skeleton if any. Otherwise, do nothing.
            var skeleton = skeletonData.FirstOrDefault(s => s.TrackingState == SkeletonTrackingState.Tracked);
            if (skeleton == null)
                return;

            // Make the faceTracker processing the data.
            FaceTrackFrame faceFrame = faceTracker.Track(kinectSensor.ColorStream.Format, colorPixelData,
                                              kinectSensor.DepthStream.Format, depthPixelData,
                                              skeleton);

            // If a face is tracked, then we can use it.
            if (faceFrame.TrackSuccessful)
            {
                var triangles = faceFrame.GetTriangles();
                // Retrieve only the Animation Units coeffs.
                var AUCoeff = faceFrame.GetAnimationUnitCoefficients();

                var jawLowerer = AUCoeff[AnimationUnit.JawLower];
                jawLowerer = jawLowerer < 0 ? 0 : jawLowerer;
                MouthScaleTransform.ScaleY = jawLowerer * 5 + 0.1;
                MouthScaleTransform.ScaleX = (AUCoeff[AnimationUnit.LipStretcher] + 1);

                LeftBrow.Y = RightBrow.Y = (AUCoeff[AnimationUnit.BrowLower]) * 40;
                RightBrowRotate.Angle = (AUCoeff[AnimationUnit.BrowRaiser] * 20);
                LeftBrowRotate.Angle = -RightBrowRotate.Angle;
                CanvasRotate.Angle = -faceFrame.Rotation.Z;
                // CanvasTranslate.X = faceFrame.Translation.X;
                // CanvasTranslate.Y = faceFrame.Translation.Y;

                if (logToFile)
                {
                    writeToFile(filename_txt.Text, faceFrame);
                }

                if (writeToOSC)
                {
                    sendOsc(osc_channel_txt.Text, faceFrame, oscWriter);
                }
            }
        }

        public void writeToFile(string fileName, FaceTrackFrame faceFrame){
            string path = Directory.GetCurrentDirectory() + @"\output";
            string target = path + "\\" + fileName;

            if (!Directory.Exists(path))
            {
                Directory.CreateDirectory(path);
            }
            if (!File.Exists(target))
            {
                using (StreamWriter file = new StreamWriter(target, true))
                {
                    string heading = "Label ";
                    heading += "TimeStamp ";
                    foreach (var fp in featurePoints){
                        heading += fp.ToString() + ".X ";
                        heading += fp.ToString() + ".Y ";
                        heading += fp.ToString() + ".Z ";
                    }
                    heading.TrimEnd(' ');
                    file.WriteLine(heading);
                }
            }

            using (StreamWriter file = new StreamWriter(target, true))
            {

                string data_string = label_txt.Text + " ";

                //Timestamp
                TimeSpan t = DateTime.UtcNow - new DateTime(1970, 1, 1);
                String time = ((long)t.TotalMilliseconds).ToString();
                data_string += time + " ";
                
                EnumIndexableCollection<FeaturePoint, Vector3DF> shapePoints = faceFrame.Get3DShape();

                foreach (FeaturePoint fp in featurePoints)
                {
                    data_string += shapePoints[fp].X + " ";
                    data_string += shapePoints[fp].Y + " ";
                    data_string += shapePoints[fp].Z + " ";
                }
                file.WriteLine(data_string);
            }
        }

        public void sendOsc(string channel, FaceTrackFrame faceFrame, UdpWriter oscWriter)
        {
            EnumIndexableCollection<FeaturePoint, Vector3DF> shapePoints = faceFrame.Get3DShape();
            /*
            Console.Out.WriteLine("Bottom: " + faceFrame.FaceRect.Bottom);
            Console.Out.WriteLine("Top: " + faceFrame.FaceRect.Top);*/


            /*var s = new Dictionary<FeaturePoint, Vector3DF>();
            foreach (FeaturePoint fp in featurePoints)
            {
                s.Add(fp, shapePoints[fp]);
            }
            string json = JsonConvert.SerializeObject(s, Formatting.Indented);
            Console.Out.Write(json);*/
            TimeSpan t = DateTime.UtcNow - new DateTime(1970, 1, 1);
            String time = ((long) t.TotalMilliseconds).ToString();

            OscBundle b = new OscBundle((ulong) t.TotalMilliseconds);
            b.AddElement(new OscElement("/" + channel + "/timestamp", time));
            foreach (FeaturePoint fp in featurePoints)
            {
                b.AddElement(new OscElement("/" + channel + "/" + fp.ToString() + "/x", shapePoints[fp].X));
                b.AddElement(new OscElement("/" + channel + "/" + fp.ToString() + "/y", shapePoints[fp].Y));
                b.AddElement(new OscElement("/" + channel + "/" + fp.ToString() + "/z", shapePoints[fp].Z));
            }
            oscWriter.Send(b);
            /*oscWriter.Send(new OscElement("/" + channel + "/timestamp" , time));
            foreach (FeaturePoint fp in featurePoints)
            {
                oscWriter.Send(new OscElement("/" + channel + "/" + fp.ToString() + "/x", shapePoints[fp].X));
                oscWriter.Send(new OscElement("/" + channel + "/" + fp.ToString() + "/y", shapePoints[fp].Y));
                oscWriter.Send(new OscElement("/" + channel + "/" + fp.ToString() + "/z", shapePoints[fp].Z));   
            }*/
            
        }

        private void TabControl_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }

        private void slider_replay_speed_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            this.replaySpeed = (int)e.NewValue;
        }
    }
}
