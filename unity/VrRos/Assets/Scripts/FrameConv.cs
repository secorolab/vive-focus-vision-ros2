// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// The single place where handedness is converted. ROS is right-handed Z-up X-forward
    /// (REP-103); Unity is left-handed Y-up Z-forward. The axis map is
    /// unity = (-ros.y, ros.z, ros.x), and because it flips handedness the quaternion's scalar
    /// part flips sign with it (a rotation of +theta about n becomes -theta about the mapped n).
    /// Every pose crossing the socket passes through here exactly once.
    /// </summary>
    public static class FrameConv
    {
        public static Vector3 RosToUnity(float x, float y, float z) => new Vector3(-y, z, x);

        public static Quaternion RosToUnity(float x, float y, float z, float w) =>
            new Quaternion(-y, z, x, -w);

        public static Vector3 UnityToRos(Vector3 p) => new Vector3(p.z, -p.x, p.y);

        public static Quaternion UnityToRos(Quaternion q) => new Quaternion(q.z, -q.x, q.y, -q.w);
    }
}
