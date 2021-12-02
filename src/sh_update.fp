/*
BSD 2 - Clause License

Copyright(c) 2013-2021, Sylvain Lefebvre, Inria, @sylefeb
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met :

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#version 420

// --------------------------------------------

in  vec4 u_Pos;
out vec4 o_PixColor;

restrict coherent uniform layout(r32f)  imageBuffer  u_Centroids;
restrict coherent uniform layout(r32ui) uimageBuffer u_AccumCenter;

uniform int u_NumCentroids;

uniform int u_ScreenWidth;

// --------------------------------------------

void main()
{
  o_PixColor = vec4(1, 0, 0, 0);

  if (gl_SampleMaskIn[0] == 0) {

    // do nothing

  } else {

    int id = int(gl_FragCoord.x) + int(gl_FragCoord.y) * u_ScreenWidth;
    if (id < u_NumCentroids) {

      vec3 ctr;
      // compute new centroid position by averaging, store
      uint num = imageLoad(u_AccumCenter, id * 4 + 3).x;
      if (num > 0) {
        // read accumulated positions, recover floating point values
        ctr.x = float(imageLoad(u_AccumCenter, id * 4 + 0).x) / float(1 << 16);
        ctr.y = float(imageLoad(u_AccumCenter, id * 4 + 1).x) / float(1 << 16);
        ctr.z = float(imageLoad(u_AccumCenter, id * 4 + 2).x) / float(1 << 16);
        ctr.xyz = ctr.xyz / float(num);
        imageStore(u_Centroids, id * 3 + 0, vec4(ctr.x));
        imageStore(u_Centroids, id * 3 + 1, vec4(ctr.y));
        imageStore(u_Centroids, id * 3 + 2, vec4(ctr.z));
      }
      o_PixColor = vec4(vec3(num) / 16.0, 0);
    }

  }

  // o_PixColor = vec4(0,1,0,0);
}

// --------------------------------------------
