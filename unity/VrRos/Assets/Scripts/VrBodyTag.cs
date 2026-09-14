// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.

using UnityEngine;

namespace VrRos
{
    /// <summary>
    /// Carries a body's index in the scene manifest, which is also its MuJoCo body index.
    ///
    /// SceneLoader puts one on each body holder; VrPointer reads it back off a raycast hit and
    /// publishes it, which is how the PC learns what the user is pointing at.
    /// </summary>
    public class VrBodyTag : MonoBehaviour
    {
        public int index = -1;
    }
}
