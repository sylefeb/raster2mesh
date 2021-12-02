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

// --------------------------------------------

#define BORDER 0.001

uniform ivec3 u_GridSize;

layout(r32ui) restrict coherent uniform uimageBuffer u_Grid;
layout(r32f)  restrict coherent uniform imageBuffer  u_GridData;
layout(r32ui) restrict coherent uniform uimageBuffer u_AccumCenter;
layout(r32ui) restrict coherent uniform uimageBuffer u_Connect;

uniform  int u_ConnectNumEntries;
uniform  int u_ComputeConnectivity;
uniform  int u_Initializing;

// --------------------------------------------

void findClosest(ivec3 ijk, vec3 pos, inout float mind, inout int mini, inout vec3 minc)
{
  ijk = min(ijk, u_GridSize - ivec3(1));
  ijk = max(ijk, ivec3(0));
  int entry = ijk.x + ijk.y * u_GridSize.x + ijk.z * u_GridSize.x * u_GridSize.y;
  // for each centroid in this grid entry
  int ptr = int(imageLoad(u_Grid, entry * 4 + 2).x);
  int num = int(imageLoad(u_Grid, entry * 4 + 1).x);
  for (int i = 0; i < num; i++) {
    vec3 ctr;
    ctr.x = imageLoad(u_GridData, (ptr + i) * 4 + 0).x;
    ctr.y = imageLoad(u_GridData, (ptr + i) * 4 + 1).x;
    ctr.z = imageLoad(u_GridData, (ptr + i) * 4 + 2).x;
    float d = length(ctr - pos);
    if (d < mind) {
      mind = d;
      mini = int(imageLoad(u_GridData, (ptr + i) * 4 + 3).x);
      minc = ctr;
    }
  }
}

// --------------------------------------------

void findTwoClosests(ivec3 ijk, vec3 pos,
  inout float mind0, inout int mini0, inout vec3 minc0,
  inout float mind1, inout int mini1, inout vec3 minc1
)
{
  ijk       = min(ijk, u_GridSize - ivec3(1));
  ijk       = max(ijk, ivec3(0));
  int entry = ijk.x + ijk.y * u_GridSize.x + ijk.z * u_GridSize.x * u_GridSize.y;
  // for each centroid in this grid entry
  int ptr   = int(imageLoad(u_Grid, entry * 4 + 2).x);
  int num   = int(imageLoad(u_Grid, entry * 4 + 1).x);
  for (int i = 0; i < num; i++) {
    vec3 ctr;
    ctr.x = imageLoad(u_GridData, (ptr + i) * 4 + 0).x;
    ctr.y = imageLoad(u_GridData, (ptr + i) * 4 + 1).x;
    ctr.z = imageLoad(u_GridData, (ptr + i) * 4 + 2).x;
    int id = int(imageLoad(u_GridData, (ptr + i) * 4 + 3).x);
    float d = length(ctr - pos);
    if (d < mind0) {
      if (id != mini0) {
        mind1 = mind0;
        mini1 = mini0;
        minc1 = minc0;
      }
      mind0 = d;
      mini0 = id;
      minc0 = ctr;
    } else if (d < mind1 && id != mini0) {
      mind1 = d;
      mini1 = id;
      minc1 = ctr;
    }
  }
}

// --------------------------------------------

void insertInSet(int id_host, int id_to_insert)
{
  int  start = id_host * u_ConnectNumEntries; // assumes this is large enough
  uint id = uint(id_to_insert) + 1;
  int iter = 0;
  while (iter < u_ConnectNumEntries) {
    uint v = imageAtomicMax(u_Connect, start + iter, id);
    if (v == 0 || v == id) {
      break;
    } else if (v < id) {
      id = v;
    }
    iter++;
  }
}

// --------------------------------------------

vec3 cvt(vec3 pos)
{
  if (gl_SampleMaskIn[0] == 0) {

    // do nothing

  } else {

    // compute grid entry
    vec3  p = 0.5 + 0.5 * pos.xyz;
    ivec3 ijk = ivec3(vec3(u_GridSize) * p);

    if (u_Initializing != 0) {
      int entry = ijk.x + ijk.y * u_GridSize.x + ijk.z * u_GridSize.x * u_GridSize.y;
      imageAtomicAdd(u_Grid, entry * 4 + 0, uint(p.x * float(1 << 16)));
      imageAtomicAdd(u_Grid, entry * 4 + 1, uint(p.y * float(1 << 16)));
      imageAtomicAdd(u_Grid, entry * 4 + 2, uint(p.z * float(1 << 16)));
      imageAtomicAdd(u_Grid, entry * 4 + 3, uint(1));
      return vec3(0.0);
    }

    // get value
    float mind = 1e9f;
    int   mini = -1;
    vec3  minc;
    float mind1 = 1e9f;
    int   mini1 = -1;
    vec3  minc1;

    if (u_ComputeConnectivity != 0) {
      // search two-closest in a 1-ring in the grid
      for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
          for (int k = -1; k <= 1; ++k) {
            findTwoClosests(ijk + ivec3(i, j, k), p,
              mind, mini, minc,
              mind1, mini1, minc1);
          }
        }
      }
    } else {
      // search closest a 1-ring in the grid
      for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
          for (int k = -1; k <= 1; ++k) {
            findClosest(ijk + ivec3(i, j, k), p, mind, mini, minc);
          }
        }
      }
    }

    if (mini > -1) {
      imageAtomicAdd(u_AccumCenter, mini * 4 + 0, uint(p.x * float(1 << 16)));
      imageAtomicAdd(u_AccumCenter, mini * 4 + 1, uint(p.y * float(1 << 16)));
      imageAtomicAdd(u_AccumCenter, mini * 4 + 2, uint(p.z * float(1 << 16)));
      imageAtomicAdd(u_AccumCenter, mini * 4 + 3, uint(1));
    }

    if (u_ComputeConnectivity != 0) {
      if (mini1 > -1) {
        // check for connectivity
        if (abs(mind - mind1) < BORDER && mini1 != mini) {
          // close enough! => insert in set
          insertInSet(mini, mini1);
          insertInSet(mini1, mini);
        }
      }
    }

    vec3 clr = vec3(mind * 16.0);
    clr = vec3(fract(mini * 1.3247), fract(mini * 7.13), fract(mini * 3.2137145));
    return clr;

  }

  return vec3(0.0);

}

// --------------------------------------------
