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

restrict coherent uniform layout(r32ui) uimageBuffer u_Table;
uniform int u_TableSize;

uniform int u_ScreenWidth;

// --------------------------------------------

void main()
{
  if (gl_SampleMaskIn[0] == 0)  {

      // do nothing

  } else {

	  int id = int(gl_FragCoord.x) + int(gl_FragCoord.y) * u_ScreenWidth;
	  if ( id < u_TableSize ) {
	    imageStore( u_Table, id, uvec4(0) );
	  }

  }

   o_PixColor = vec4(0,1,0,0);
}

// --------------------------------------------
