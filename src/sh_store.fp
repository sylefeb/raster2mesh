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

uniform layout(r32f) imageBuffer u_Centroids;
uniform int u_NumCentroids;

restrict coherent uniform layout(r32ui) uimageBuffer u_Grid;
uniform ivec3 u_GridSize;

restrict coherent uniform layout(r32f)  imageBuffer u_GridData;

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

      vec4 ctr;
      ctr.x = imageLoad(u_Centroids, id * 4 + 0).x;
      ctr.y = imageLoad(u_Centroids, id * 4 + 1).x;
      ctr.z = imageLoad(u_Centroids, id * 4 + 2).x;
      ctr.w = imageLoad(u_Centroids, id * 4 + 3).x;
      // compute grid entry
      ivec3 ijk = ivec3(vec3(u_GridSize) * ctr.xyz);
      ijk       = min(ijk, u_GridSize - ivec3(1));
      ijk       = max(ijk, ivec3(0));
      // store
      int entry = ijk.x + ijk.y * u_GridSize.x + ijk.z * u_GridSize.x * u_GridSize.y;
      int pos   = int(imageLoad(u_Grid, entry * 3 + 2).x
                    + imageAtomicAdd(u_Grid, entry * 3 + 1, uint(1)));
      if (pos < u_NumCentroids) {
        imageStore(u_GridData, pos * 4 + 0, vec4(ctr.x));
        imageStore(u_GridData, pos * 4 + 1, vec4(ctr.y));
        imageStore(u_GridData, pos * 4 + 2, vec4(ctr.z));
        imageStore(u_GridData, pos * 4 + 3, vec4(id));
        o_PixColor = vec4(float(pos & 255u) / 255.0, float(pos / 255u) / 255.0, 0, 0);
      } else {
        o_PixColor = vec4(0, 0, 1, 0);
      }
    }

  }

}

// --------------------------------------------
