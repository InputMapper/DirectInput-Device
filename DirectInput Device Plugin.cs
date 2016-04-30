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
                this.InputChannels.Add(value as InputChannel);
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
        public InputChannelTypes.JoyAxis ax { get; }
        public InputChannelTypes.JoyAxis ay { get; }
        public InputChannelTypes.JoyAxis az { get; }

        public InputChannelTypes.JoyAxis aax { get; }
        public InputChannelTypes.JoyAxis aay { get; }
        public InputChannelTypes.JoyAxis aaz { get; }

        public InputChannelTypes.JoyAxis avx { get; }
        public InputChannelTypes.JoyAxis avy { get; }
        public InputChannelTypes.JoyAxis avz { get; }

        public InputChannelTypes.JoyAxis fx { get; }
        public InputChannelTypes.JoyAxis fy { get; }
        public InputChannelTypes.JoyAxis fz { get; }

        public InputChannelTypes.JoyAxis rx { get; }
        public InputChannelTypes.JoyAxis ry { get; }
        public InputChannelTypes.JoyAxis rz { get; }

        public InputChannelTypes.JoyAxis tx { get; }
        public InputChannelTypes.JoyAxis ty { get; }
        public InputChannelTypes.JoyAxis tz { get; }

        public InputChannelTypes.JoyAxis vx { get; }
        public InputChannelTypes.JoyAxis vy { get; }
        public InputChannelTypes.JoyAxis vz { get; }

        public InputChannelTypes.JoyAxis x { get; }
        public InputChannelTypes.JoyAxis y { get; }
        public InputChannelTypes.JoyAxis z { get; }

        public InputChannelTypes.JoyAxis Slider1 { get; }
        public InputChannelTypes.JoyAxis Slider2 { get; }
        public InputChannelTypes.JoyAxis Slider3 { get; }
        public InputChannelTypes.JoyAxis Slider4 { get; }
        public InputChannelTypes.JoyAxis Slider5 { get; }
        public InputChannelTypes.JoyAxis Slider6 { get; }
        public InputChannelTypes.JoyAxis Slider7 { get; }
        public InputChannelTypes.JoyAxis Slider8 { get; }
        public InputChannelTypes.JoyAxis Slider9 { get; }
        public InputChannelTypes.JoyAxis Slider10 { get; }

        public InputChannelTypes.Button b1 { get; }
        public InputChannelTypes.Button b2 { get; }
        public InputChannelTypes.Button b3 { get; }
        public InputChannelTypes.Button b4 { get; }
        public InputChannelTypes.Button b5 { get; }
        public InputChannelTypes.Button b6 { get; }
        public InputChannelTypes.Button b7 { get; }
        public InputChannelTypes.Button b8 { get; }
        public InputChannelTypes.Button b9 { get; }
        public InputChannelTypes.Button b10 { get; }
        public InputChannelTypes.Button b11 { get; }
        public InputChannelTypes.Button b12 { get; }
        public InputChannelTypes.Button b13 { get; }
        public InputChannelTypes.Button b14 { get; }
        public InputChannelTypes.Button b15 { get; }
        public InputChannelTypes.Button b16 { get; }
        public InputChannelTypes.Button b17 { get; }
        public InputChannelTypes.Button b18 { get; }
        public InputChannelTypes.Button b19 { get; }
        public InputChannelTypes.Button b20 { get; }

        public InputChannelTypes.Button pov1up { get; }
        public InputChannelTypes.Button pov1down { get; }
        public InputChannelTypes.Button pov1left { get; }
        public InputChannelTypes.Button pov1right { get; }

        public InputChannelTypes.Button pov2up { get; }
        public InputChannelTypes.Button pov2down { get; }
        public InputChannelTypes.Button pov2left { get; }
        public InputChannelTypes.Button pov2right { get; }

        public DIdevice()
        {
            ax = new InputChannelTypes.JoyAxis("Acceleration X");
            ay = new InputChannelTypes.JoyAxis("Acceleration Y");
            az = new InputChannelTypes.JoyAxis("Acceleration Z");

            aax = new InputChannelTypes.JoyAxis("Angular Acceleration X");
            aay = new InputChannelTypes.JoyAxis("Angular Acceleration Y");
            aaz = new InputChannelTypes.JoyAxis("Angular Acceleration Z");

            avx = new InputChannelTypes.JoyAxis("Angular Velocity X");
            avy = new InputChannelTypes.JoyAxis("Angular Velocity Y");
            avz = new InputChannelTypes.JoyAxis("Angular Velocity Z");

            fx = new InputChannelTypes.JoyAxis("Force X");
            fy = new InputChannelTypes.JoyAxis("Force Y");
            fz = new InputChannelTypes.JoyAxis("Force Z");

            rx = new InputChannelTypes.JoyAxis("Rotation X");
            ry = new InputChannelTypes.JoyAxis("Rotation Y");
            rz = new InputChannelTypes.JoyAxis("Rotation Z");

            vx = new InputChannelTypes.JoyAxis("Velocity X");
            vy = new InputChannelTypes.JoyAxis("Velocity Y");
            vz = new InputChannelTypes.JoyAxis("Velocity Z");

            tx = new InputChannelTypes.JoyAxis("Torque X");
            ty = new InputChannelTypes.JoyAxis("Torque Y");
            tz = new InputChannelTypes.JoyAxis("Torque Z");

            x = new InputChannelTypes.JoyAxis("X");
            y = new InputChannelTypes.JoyAxis("Y");
            z = new InputChannelTypes.JoyAxis("Z");

            Slider1 = new InputChannelTypes.JoyAxis("Slider 1");
            Slider2 = new InputChannelTypes.JoyAxis("Slider 2");
            Slider3 = new InputChannelTypes.JoyAxis("Slider 3");
            Slider4 = new InputChannelTypes.JoyAxis("Slider 4");
            Slider5 = new InputChannelTypes.JoyAxis("Slider 5");
            Slider6 = new InputChannelTypes.JoyAxis("Slider 6");
            Slider7 = new InputChannelTypes.JoyAxis("Slider 7");
            Slider8 = new InputChannelTypes.JoyAxis("Slider 8");
            Slider9 = new InputChannelTypes.JoyAxis("Slider 9");
            Slider10 = new InputChannelTypes.JoyAxis("Slider 10");

            b1 = new InputChannelTypes.Button("Button 1");
            b2 = new InputChannelTypes.Button("Button 2");
            b3 = new InputChannelTypes.Button("Button 3");
            b4 = new InputChannelTypes.Button("Button 4");
            b5 = new InputChannelTypes.Button("Button 5");
            b6 = new InputChannelTypes.Button("Button 6");
            b7 = new InputChannelTypes.Button("Button 7");
            b8 = new InputChannelTypes.Button("Button 8");
            b9 = new InputChannelTypes.Button("Button 9");
            b10 = new InputChannelTypes.Button("Button 10");
            b11 = new InputChannelTypes.Button("Button 11");
            b12 = new InputChannelTypes.Button("Button 12");
            b13 = new InputChannelTypes.Button("Button 13");
            b14 = new InputChannelTypes.Button("Button 14");
            b15 = new InputChannelTypes.Button("Button 15");
            b16 = new InputChannelTypes.Button("Button 16");
            b17 = new InputChannelTypes.Button("Button 17");
            b18 = new InputChannelTypes.Button("Button 18");
            b19 = new InputChannelTypes.Button("Button 19");
            b20 = new InputChannelTypes.Button("Button 20");

            pov1up = new InputChannelTypes.Button("POV 1 Up");
            pov1down = new InputChannelTypes.Button("POV 1 Down");
            pov1left = new InputChannelTypes.Button("POV 1 Left");
            pov1right = new InputChannelTypes.Button("POV 1 Right");

            pov2up = new InputChannelTypes.Button("POV 2 Up");
            pov2down = new InputChannelTypes.Button("POV 2 Down");
            pov2left = new InputChannelTypes.Button("POV 2 Left");
            pov2right = new InputChannelTypes.Button("POV 2 Right");
        }

    }

}
