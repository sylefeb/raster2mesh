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

restrict coherent uniform layout(r32ui) uimageBuffer u_Triangles;
restrict coherent uniform layout(r32ui) uimageBuffer u_Connect;
restrict coherent uniform layout(r32ui) uimageBuffer u_Counter;

uniform  uint u_ConnectNumEntries;
uniform  int  u_NumCentroids;
uniform  uint u_ScreenWidth;

// --------------------------------------------

void main()
{
   o_PixColor = vec4(1,0,0,0);

  if (gl_SampleMaskIn[0] == 0)  {

      // do nothing

  } else {

      bool diamond = false;
      uint min_diam = uint(1<<20);
      uint max_diam = 0;

	  uint id0 = uint(gl_FragCoord.x) + uint(gl_FragCoord.y) * u_ScreenWidth;
	  if ( id0 < u_NumCentroids ) {

		uint start0 = id0 * u_ConnectNumEntries;
		for (uint i0 = 0 ; i0 < u_ConnectNumEntries ; i0 ++ ) {

		  uint id1 = imageLoad( u_Connect, int(start0 + i0) ).x;
		  if (id1 == 0) break; else id1 --;
		  if (id1 == id0) continue; // TODO useless

	      // search triangles
		  int found = 0;

		  uint start1 = id1 * u_ConnectNumEntries;
		  for (uint i1 = 0 ; i1 < u_ConnectNumEntries ; i1 ++ ) {

		    uint id2 = imageLoad( u_Connect, int(start1 + i1) ).x;
		    if (id2 == 0) break; else id2 --;
		    if (id2 == id0) continue;
		    if (id2 == id1) continue; // TODO useless

  		    uint start2 = id2 * u_ConnectNumEntries;

		    for (uint i2 = 0 ; i2 < u_ConnectNumEntries ; i2 ++ ) {

		       uint id3 = imageLoad( u_Connect, int(start2 + i2) ).x;
		       if (id3 == 0) break; else id3 --;
			   if (id3 == id0) {
				 // triangle found!!
				 found ++;
			     if (id0 < id1 && id1 < id2) {
					 // insert the triangle
					 uint ptr = imageAtomicAdd( u_Counter, 0, 1u );
					 imageStore(u_Triangles, int(ptr*3+0), uvec4(id0));
					 imageStore(u_Triangles, int(ptr*3+1), uvec4(id1));
					 imageStore(u_Triangles, int(ptr*3+2), uvec4(id2));
				 }
			   }

			} // i2
		  } // i1

  	      // diamond?
		  if ( found < 2 ) {
		    min_diam = min(min_diam,id1);
		    max_diam = max(max_diam,id1);
			diamond  = true;
  		  }
		} // i0

		if (diamond && min_diam < max_diam) {
		  uint id1    = min_diam;
		  uint start1 = id1 * u_ConnectNumEntries;
		  for (uint i1 = 0 ; i1 < u_ConnectNumEntries ; i1 ++ ) {

			uint id2 = imageLoad( u_Connect, int(start1 + i1) ).x;
			if (id2 == 0) break; else id2 --;
			if (id2 == id0) continue;

  			  uint start2 = id2 * u_ConnectNumEntries;

		      for (uint i2 = 0 ; i2 < u_ConnectNumEntries ; i2 ++ ) {

	            uint id3 = imageLoad( u_Connect, int(start2 + i2) ).x;
	            if (id3 == 0) break; else id3 --;
			    if (id3 != max_diam) continue;

				  // diamond found!!
				  uint ptr = imageAtomicAdd( u_Counter, 0, 2u );
				  imageStore(u_Triangles, int(ptr*3+0), uvec4(id0));
				  imageStore(u_Triangles, int(ptr*3+1), uvec4(id1));
				  imageStore(u_Triangles, int(ptr*3+2), uvec4(id2));
				  ptr ++;
				  imageStore(u_Triangles, int(ptr*3+0), uvec4(id0));
				  imageStore(u_Triangles, int(ptr*3+1), uvec4(id2));
				  imageStore(u_Triangles, int(ptr*3+2), uvec4(id3));

  			  } // i2
		    } // i1
	    } // diamond?

      o_PixColor = vec4(0,1,0,0);

	} // centroid

  } // mask

} // main

// --------------------------------------------
