using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ODIF;
using SlimDX;
using SlimDX.DirectInput;
using System.Reflection;
using System.Threading;
using ODIF.Extensions;
namespace DirectInput_Device
{

    [PluginInfo(
        PluginName = "DirectInput Controller Input",
        PluginDescription = "",
        PluginID = 33,
        PluginAuthorName = "InputMapper",
        PluginAuthorEmail = "jhebbel@gmail.com",
        PluginAuthorURL = "http://inputmapper.com",
        PluginIconPath = @"pack://application:,,,/DirecInput Device;component/Resources/GenericGamepad.png"
        )]
    public class DirectInput_Device_Plugin :InputDevicePlugin
    {
        DirectInput directInput = new DirectInput();
        SlimDX.DirectInput.Joystick gamepad;
        SlimDX.DirectInput.JoystickState state;

        public DirectInput_Device_Plugin()
        {
            checkForDevices();
            Global.HardwareChangeDetected += Global_HardwareChangeDetected;
        }

        private void Global_HardwareChangeDetected(object sender, System.Management.EventArrivedEventArgs e)
        {
            checkForDevices();
        }

        private void checkForDevices()
        {
            lock (Devices)
            {
                List<DeviceInstance> directInputList = new List<DeviceInstance>();
                directInputList.AddRange(directInput.GetDevices(DeviceClass.GameController, DeviceEnumerationFlags.AttachedOnly));

                foreach (DeviceInstance controller in directInputList)
                {
                    if (Devices.Where(d => (d as DirectInput_Device).device.InstanceGuid == controller.InstanceGuid).Count() == 0)
                        Devices.Add(new DirectInput_Device(directInput, controller));
                }

                foreach (DirectInput_Device device in Devices.ToList())
                {
                    if (directInputList.Where(d=> d.InstanceGuid == device.device.InstanceGuid).Count() == 0)
                        Devices.Remove(device);
                }
            }
        }

    }
    public class DirectInput_Device : InputDevice
    {
        internal DeviceInstance device;
        SlimDX.DirectInput.Joystick gamepad;
        DirectInput directInput;
        DIdevice deviceWrapper;
        Thread poolingThread;
        private bool stopThread = false;

        public DirectInput_Device(DirectInput directInput, DeviceInstance device)
        {
            this.StatusIcon = Properties.Resources.GenericGamepad.ToImageSource();
            this.directInput = directInput;
            this.device = device;
            this.deviceWrapper = new DIdevice();
            gamepad = new SlimDX.DirectInput.Joystick(directInput, device.InstanceGuid);

            this.DeviceName = gamepad.Information.ProductName;
            gamepad.Acquire();


            foreach (DeviceObjectInstance deviceObject in gamepad.GetObjects())
            {
                if ((deviceObject.ObjectType & ObjectDeviceType.Axis) != 0)
                    gamepad.GetObjectPropertiesById((int)deviceObject.ObjectType).SetRange(-1000, 1000);
            }

            foreach (var property in deviceWrapper.GetType().GetProperties())
            {
                PropertyInfo propertyS = deviceWrapper.GetType().GetProperty(property.Name);
                var value = property.GetValue(deviceWrapper, null);
                this.Channels.Add(value as DeviceChannel);
            }

            poolingThread = new Thread(ListenerThread);
            poolingThread.Start();

        }

        public void ListenerThread()
        {
            SlimDX.DirectInput.JoystickState state;
            
            while (!stopThread && !Global.IsShuttingDown)
            {
                Thread.Sleep(2);
                state = gamepad.GetCurrentState();

                deviceWrapper.ax.Value = state.AccelerationX / 1000f;
                deviceWrapper.ay.Value = state.AccelerationY / 1000f;
                deviceWrapper.az.Value = state.AccelerationZ / 1000f;

                deviceWrapper.aax.Value = state.AngularAccelerationX / 1000f;
                deviceWrapper.aay.Value = state.AngularAccelerationY / 1000f;
                deviceWrapper.aaz.Value = state.AngularAccelerationZ / 1000f;

                deviceWrapper.avx.Value = state.AngularVelocityX / 1000f;
                deviceWrapper.avy.Value = state.AngularVelocityY / 1000f;
                deviceWrapper.avz.Value = state.AngularVelocityZ / 1000f;

                deviceWrapper.fx.Value = state.ForceX / 1000f;
                deviceWrapper.fy.Value = state.ForceY / 1000f;
                deviceWrapper.fz.Value = state.ForceZ / 1000f;

                deviceWrapper.rx.Value = state.RotationX / 1000f;
                deviceWrapper.ry.Value = state.RotationY / 1000f;
                deviceWrapper.rz.Value = state.RotationZ / 1000f;

                deviceWrapper.tx.Value = state.TorqueX / 1000f;
                deviceWrapper.ty.Value = state.TorqueY / 1000f;
                deviceWrapper.tz.Value = state.TorqueZ / 1000f;

                deviceWrapper.vx.Value = state.VelocityX / 1000f;
                deviceWrapper.vy.Value = state.VelocityY / 1000f;
                deviceWrapper.vz.Value = state.VelocityZ / 1000f;

                deviceWrapper.x.Value = state.X / 1000f;
                deviceWrapper.y.Value = state.Y / 1000f;
                deviceWrapper.z.Value = state.Z / 1000f;

                deviceWrapper.b1.Value = state.IsPressed(0);
                deviceWrapper.b2.Value = state.IsPressed(1);
                deviceWrapper.b3.Value = state.IsPressed(2);
                deviceWrapper.b4.Value = state.IsPressed(3);
                deviceWrapper.b5.Value = state.IsPressed(4);
                deviceWrapper.b6.Value = state.IsPressed(5);
                deviceWrapper.b7.Value = state.IsPressed(6);
                deviceWrapper.b8.Value = state.IsPressed(7);
                deviceWrapper.b9.Value = state.IsPressed(8);
                deviceWrapper.b10.Value = state.IsPressed(9);
                deviceWrapper.b11.Value = state.IsPressed(10);
                deviceWrapper.b12.Value = state.IsPressed(11);
                deviceWrapper.b13.Value = state.IsPressed(12);
                deviceWrapper.b14.Value = state.IsPressed(13);
                deviceWrapper.b15.Value = state.IsPressed(14);
                deviceWrapper.b16.Value = state.IsPressed(15);
                deviceWrapper.b17.Value = state.IsPressed(16);
                deviceWrapper.b18.Value = state.IsPressed(17);
                deviceWrapper.b19.Value = state.IsPressed(18);
                deviceWrapper.b20.Value = state.IsPressed(19);

                var pov = (state.GetPointOfViewControllers()[0]);
                deviceWrapper.pov1up.Value = (0 <= pov && pov <= 4500 || 31500 <= pov);
                deviceWrapper.pov1down.Value = (13500 <= pov && pov <= 22500);
                deviceWrapper.pov1left.Value = (22500 <= pov && pov <= 31500);
                deviceWrapper.pov1right.Value = (4500 <= pov && pov <= 13500);

                pov = (state.GetPointOfViewControllers()[1]);
                deviceWrapper.pov2up.Value = (0 <= pov && pov <= 4500 || 31500 <= pov);
                deviceWrapper.pov2down.Value = (13500 <= pov && pov <= 22500);
                deviceWrapper.pov2left.Value = (22500 <= pov && pov <= 31500);
                deviceWrapper.pov2right.Value = (4500 <= pov && pov <= 13500);

                int[] sliders = state.GetSliders();
                if (sliders.Count() > 0)
                    deviceWrapper.Slider1.Value = sliders[0] / 65535f;
                if (sliders.Count() > 1)
                    deviceWrapper.Slider1.Value = sliders[1] / 65535f;
                if (sliders.Count() > 2)
                    deviceWrapper.Slider1.Value = sliders[2] / 65535f;
                if (sliders.Count() > 3)
                    deviceWrapper.Slider1.Value = sliders[3] / 65535f;
                if (sliders.Count() > 4)
                    deviceWrapper.Slider1.Value = sliders[4] / 65535f;
                if (sliders.Count() > 5)
                    deviceWrapper.Slider1.Value = sliders[5] / 65535f;
                if (sliders.Count() > 6)
                    deviceWrapper.Slider1.Value = sliders[6] / 65535f;
                if (sliders.Count() > 7)
                    deviceWrapper.Slider1.Value = sliders[7] / 65535f;
                if (sliders.Count() > 8)
                    deviceWrapper.Slider1.Value = sliders[8] / 65535f;
                if (sliders.Count() > 9)
                    deviceWrapper.Slider1.Value = sliders[9] / 65535f;
            }
        }

        protected override void Dispose(bool disposing)
        {
            stopThread = true;

            if (poolingThread != null)
                poolingThread.Abort();

            gamepad.Unacquire();

            base.Dispose(disposing);
        }
    }
    internal class DIdevice
    {
        public JoyAxis ax { get; }
        public JoyAxis ay { get; }
        public JoyAxis az { get; }

        public JoyAxis aax { get; }
        public JoyAxis aay { get; }
        public JoyAxis aaz { get; }

        public JoyAxis avx { get; }
        public JoyAxis avy { get; }
        public JoyAxis avz { get; }

        public JoyAxis fx { get; }
        public JoyAxis fy { get; }
        public JoyAxis fz { get; }

        public JoyAxis rx { get; }
        public JoyAxis ry { get; }
        public JoyAxis rz { get; }

        public JoyAxis tx { get; }
        public JoyAxis ty { get; }
        public JoyAxis tz { get; }

        public JoyAxis vx { get; }
        public JoyAxis vy { get; }
        public JoyAxis vz { get; }

        public JoyAxis x { get; }
        public JoyAxis y { get; }
        public JoyAxis z { get; }

        public JoyAxis Slider1 { get; }
        public JoyAxis Slider2 { get; }
        public JoyAxis Slider3 { get; }
        public JoyAxis Slider4 { get; }
        public JoyAxis Slider5 { get; }
        public JoyAxis Slider6 { get; }
        public JoyAxis Slider7 { get; }
        public JoyAxis Slider8 { get; }
        public JoyAxis Slider9 { get; }
        public JoyAxis Slider10 { get; }

        public Button b1 { get; }
        public Button b2 { get; }
        public Button b3 { get; }
        public Button b4 { get; }
        public Button b5 { get; }
        public Button b6 { get; }
        public Button b7 { get; }
        public Button b8 { get; }
        public Button b9 { get; }
        public Button b10 { get; }
        public Button b11 { get; }
        public Button b12 { get; }
        public Button b13 { get; }
        public Button b14 { get; }
        public Button b15 { get; }
        public Button b16 { get; }
        public Button b17 { get; }
        public Button b18 { get; }
        public Button b19 { get; }
        public Button b20 { get; }

        public Button pov1up { get; }
        public Button pov1down { get; }
        public Button pov1left { get; }
        public Button pov1right { get; }

        public Button pov2up { get; }
        public Button pov2down { get; }
        public Button pov2left { get; }
        public Button pov2right { get; }

        public DIdevice()
        {
            ax = new JoyAxis("Acceleration X", DataFlowDirection.Input);
            ay = new JoyAxis("Acceleration Y", DataFlowDirection.Input);
            az = new JoyAxis("Acceleration Z", DataFlowDirection.Input);

            aax = new JoyAxis("Angular Acceleration X", DataFlowDirection.Input);
            aay = new JoyAxis("Angular Acceleration Y", DataFlowDirection.Input);
            aaz = new JoyAxis("Angular Acceleration Z", DataFlowDirection.Input);

            avx = new JoyAxis("Angular Velocity X", DataFlowDirection.Input);
            avy = new JoyAxis("Angular Velocity Y", DataFlowDirection.Input);
            avz = new JoyAxis("Angular Velocity Z", DataFlowDirection.Input);

            fx = new JoyAxis("Force X", DataFlowDirection.Input);
            fy = new JoyAxis("Force Y", DataFlowDirection.Input);
            fz = new JoyAxis("Force Z", DataFlowDirection.Input);

            rx = new JoyAxis("Rotation X", DataFlowDirection.Input);
            ry = new JoyAxis("Rotation Y", DataFlowDirection.Input);
            rz = new JoyAxis("Rotation Z", DataFlowDirection.Input);

            vx = new JoyAxis("Velocity X", DataFlowDirection.Input);
            vy = new JoyAxis("Velocity Y", DataFlowDirection.Input);
            vz = new JoyAxis("Velocity Z", DataFlowDirection.Input);

            tx = new JoyAxis("Torque X", DataFlowDirection.Input);
            ty = new JoyAxis("Torque Y", DataFlowDirection.Input);
            tz = new JoyAxis("Torque Z", DataFlowDirection.Input);

            x = new JoyAxis("X", DataFlowDirection.Input);
            y = new JoyAxis("Y", DataFlowDirection.Input);
            z = new JoyAxis("Z", DataFlowDirection.Input);

            Slider1 = new JoyAxis("Slider 1", DataFlowDirection.Input);
            Slider2 = new JoyAxis("Slider 2", DataFlowDirection.Input);
            Slider3 = new JoyAxis("Slider 3", DataFlowDirection.Input);
            Slider4 = new JoyAxis("Slider 4", DataFlowDirection.Input);
            Slider5 = new JoyAxis("Slider 5", DataFlowDirection.Input);
            Slider6 = new JoyAxis("Slider 6", DataFlowDirection.Input);
            Slider7 = new JoyAxis("Slider 7", DataFlowDirection.Input);
            Slider8 = new JoyAxis("Slider 8", DataFlowDirection.Input);
            Slider9 = new JoyAxis("Slider 9", DataFlowDirection.Input);
            Slider10 = new JoyAxis("Slider 10", DataFlowDirection.Input);

            b1 = new Button("Button 1", DataFlowDirection.Input);
            b2 = new Button("Button 2", DataFlowDirection.Input);
            b3 = new Button("Button 3", DataFlowDirection.Input);
            b4 = new Button("Button 4", DataFlowDirection.Input);
            b5 = new Button("Button 5", DataFlowDirection.Input);
            b6 = new Button("Button 6", DataFlowDirection.Input);
            b7 = new Button("Button 7", DataFlowDirection.Input);
            b8 = new Button("Button 8", DataFlowDirection.Input);
            b9 = new Button("Button 9", DataFlowDirection.Input);
            b10 = new Button("Button 10", DataFlowDirection.Input);
            b11 = new Button("Button 11", DataFlowDirection.Input);
            b12 = new Button("Button 12", DataFlowDirection.Input);
            b13 = new Button("Button 13", DataFlowDirection.Input);
            b14 = new Button("Button 14", DataFlowDirection.Input);
            b15 = new Button("Button 15", DataFlowDirection.Input);
            b16 = new Button("Button 16", DataFlowDirection.Input);
            b17 = new Button("Button 17", DataFlowDirection.Input);
            b18 = new Button("Button 18", DataFlowDirection.Input);
            b19 = new Button("Button 19", DataFlowDirection.Input);
            b20 = new Button("Button 20", DataFlowDirection.Input);

            pov1up = new Button("POV 1 Up", DataFlowDirection.Input);
            pov1down = new Button("POV 1 Down", DataFlowDirection.Input);
            pov1left = new Button("POV 1 Left", DataFlowDirection.Input);
            pov1right = new Button("POV 1 Right", DataFlowDirection.Input);

            pov2up = new Button("POV 2 Up", DataFlowDirection.Input);
            pov2down = new Button("POV 2 Down", DataFlowDirection.Input);
            pov2left = new Button("POV 2 Left", DataFlowDirection.Input);
            pov2right = new Button("POV 2 Right", DataFlowDirection.Input);
        }

    }

}
