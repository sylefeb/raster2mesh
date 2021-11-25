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

restrict coherent uniform layout(r32ui) uimageBuffer u_Counter;
restrict coherent uniform layout(r32ui) uimageBuffer u_Grid;
uniform int u_GridSize;

uniform int u_ScreenWidth;

// --------------------------------------------

void main()
{
   o_PixColor = vec4(1,0,0,0);

  if (gl_SampleMaskIn[0] == 0)  {

      // do nothing

  } else {

	  int id = int(gl_FragCoord.x) + int(gl_FragCoord.y) * u_ScreenWidth;
	  if ( id < u_GridSize ) {
      // read counter
      uint tot = imageLoad(u_Grid, id*3 + 0 ).x;
      // increment allocation counter
      uint ptr = imageAtomicAdd( u_Counter, 0, tot );
      imageStore(u_Grid, id*3 + 2, uvec4(ptr) );
      // debug output
      o_PixColor = vec4(float(ptr&255u)/255.0,float(ptr/255u)/255.0,0,0);
	  }

  }

}

// --------------------------------------------
