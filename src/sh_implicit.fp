#version 420
#extension GL_ARB_shader_image_load_store : enable
#extension GL_ARB_gpu_shader5 : enable

// The MIT License
// Copyright (c) 2013 Inigo Quilez
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// https://www.youtube.com/c/InigoQuilez
// https://iquilezles.org

// @sylefeb, grabbed from https://www.shadertoy.com/view/ld2GRz
//           heavily modified for CVT remeshing demo


// Using polynomial fallof of degree 5 for bounded metaballs, which produce smooth normals
// unlike the cubic (smoothstep) based fallofs recommended in literature (such as John Hart).

// The quintic polynomial p(x) = 6x5 - 15x4 + 10x3 has zero first and second derivatives in
// its corners. The maxium slope p''(x)=0 happens in the middle x=1/2, and its value is
// p'(1/2) = 15/8. Therefore the  minimum distance to a metaball (in metaball canonical
// coordinates) is at least 8/15 = 0.533333 (see line 63).

// This shader uses bounding spheres for each ball so that rays traver much faster when far
// or outside their radius of influence.

#define numballs 5

//----------------------------------------------------------------

float hash1(float n)
{
  return fract(sin(n) * 43758.5453123);
}

vec2 hash2(float n)
{
  return fract(sin(vec2(n, n + 1.0)) * vec2(43758.5453123, 22578.1459123));
}

vec3 hash3(float n)
{
  return fract(sin(vec3(n, n + 1.0, n + 2.0)) * vec3(43758.5453123, 22578.1459123, 19642.3490423));
}

//----------------------------------------------------------------

vec4 blobs[numballs];

float sdMetaBalls(vec3 pos)
{
  float m = 0.0;
  float p = 0.0;
  float dmin = 1e20;
  float h = 1.0; // track Lipschitz constant

  for (int i = 0; i < numballs; i++) 	{
    // bounding sphere for ball
    float db = length(blobs[i].xyz - pos);
    if (db < blobs[i].w) {
      float x = db / blobs[i].w;
      p += 1.0 - x * x * x * (x * (x * 6.0 - 15.0) + 10.0);
      m += 1.0;
      h = max(h,0.5333 * blobs[i].w);
    } else { // bouncing sphere distance
      dmin = min(dmin, db - blobs[i].w);
    }
  }
  float d = dmin + 0.1;
  if (m > 0.5) 	{
    float th = 0.2;
    d = 0.5 * h * (th - p);
  }
  return d;
}

const float precis  = 0.01;
const float epsilon = 0.001;

float map(in vec3 p)
{
  return sdMetaBalls(p);
}

vec3 norMetaBalls(vec3 pos)
{
  vec3 nor = vec3(0.0, 0.0001, 0.0);

  for (int i = 0; i < numballs; i++) 	{
    float db = length(blobs[i].xyz - pos);
    float x = clamp(db / blobs[i].w, 0.0, 1.0);
    float p = x * x * (30.0 * x * x - 60.0 * x + 30.0);
    nor += normalize(pos - blobs[i].xyz) * p / blobs[i].w;
  }

  return normalize(nor);
}

vec2 intersect(in vec3 ro, in vec3 rd)
{
  const float maxd = 10.0;
  float t = 0.0;
  float e = map(ro);
  float h = abs(e);
  // find next intersection
  int i = 0;
  const int maxiter = 1024;
  for ( ; i < maxiter; i++) {
    t += max(epsilon,abs(h));
    h  = map(ro + rd * t);
    if (  (h < -precis && e >  precis)
       || (h >  precis && e < -precis)
       || t > maxd) break;
  }
  float m = 1.0;
  if (t > maxd || i == maxiter) { m = -1.0; }
  return vec2(t, m);
}

uniform vec2  iResolution;
uniform float iTime;
uniform int   u_Dir;

out vec4      o_PixColor;

//-----------------------------------------------------
// include the meshing stuff
#include "sh_cvt.fp"
//-----------------------------------------------------

void main()
{
  //-----------------------------------------------------
  // input
  //-----------------------------------------------------

  vec2 q = gl_FragCoord.xy / iResolution.y;

  vec2 m = vec2(0.5);

  vec2  poff = vec2(0.0);
  float toff = 0.0;
  //-----------------------------------------------------
  // animate scene
  //-----------------------------------------------------
  float time = iTime + toff;

  // move metaballs
  for (int i = 0; i < numballs; i++) {
    float h = float(i) / 8.0;
    blobs[i].xyz = 2.0 * sin(6.2831 * hash3(h * 1.17) + hash3(h * 13.7) * time);
    blobs[i].w   = 1.7 + 0.9 * sin(6.28 * hash1(h * 23.13));
  }

  //-----------------------------------------------------
  // orthographic camera
  //-----------------------------------------------------
  vec3 ro, rd;
  vec3 vw = vec3((q - 0.5) * 10.0, 5.0);
  if (u_Dir == 0) {
    ro = vw.xyz;
    rd = vec3(0.0, 0.0, -1.0);
  } else if (u_Dir == 1) {
    ro = vw.zxy;
    rd = vec3(-1.0, 0.0, 0.0);
  } else {
    ro = vw.yzx;
    rd = vec3(0.0, -1.0, 0.0);
  }

  //-----------------------------------------------------
  // render
  //-----------------------------------------------------

  // background
  o_PixColor = vec4(0.0, 0.0, 0.0, 1.0);
  // raymarch
  for (int n = 0; n < 4; ++n) {
    vec2 tmat = intersect(ro, rd);
    if (tmat.y > -0.5) {
      // geometry
      vec3 pos = ro  + tmat.x * rd;
      ro       = pos; // restart from there
      vec3 nrm = norMetaBalls(pos);
      // give point to CVT mesher
      cvt(pos / 4.0);
      // for display
      o_PixColor.xyz = nrm * 0.5 + 0.5;
    } else {
      break;
    }
  }

}

//-----------------------------------------------------
