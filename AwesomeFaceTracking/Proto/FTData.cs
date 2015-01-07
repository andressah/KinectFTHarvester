﻿// Classes and structures being serialized

// Generated by ProtocolBuffer
// - a pure c# code generation implementation of protocol buffers
// Report bugs to: https://silentorbit.com/protobuf/

// DO NOT EDIT
// This file will be overwritten when CodeGenerator is run.
// To make custom modifications, edit the .proto file and add //:external before the message line
// then write the code and the changes in a separate file.
using System;
using System.Collections.Generic;

namespace Example
{
    public partial class FaceTrackFrame
    {
        public long Timestamp { get; set; }

        public string Label { get; set; }

        public List<Example.Point> TrackedPoint { get; set; }

    }

    public partial class Point
    {
        public double X { get; set; }

        public double Y { get; set; }

        public double Z { get; set; }

    }

}
