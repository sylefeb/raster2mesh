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
// --------------------------------------------------------------

#include <iostream>
#include <ctime>
#include <cmath>

// --------------------------------------------------------------

#include <LibSL/LibSL_gl4.h>
#include <imgui.h>
#include <LibSL/UIHelpers/BindImGui.h>

#include "sh_clear.h"
#include "sh_init.h"
#include "sh_count.h"
#include "sh_allocate.h"
#include "sh_store.h"
#include "sh_update.h"
#include "sh_gentris.h"
#include "sh_mesh.h"
#include "sh_raster.h"
#include "sh_implicit.h"

AutoBindShader::sh_clear      g_ShClear;
AutoBindShader::sh_count      g_ShCount;
AutoBindShader::sh_allocate   g_ShAllocate;
AutoBindShader::sh_store      g_ShStore;
AutoBindShader::sh_update     g_ShUpdate;
AutoBindShader::sh_gentris    g_ShGenTris;
AutoBindShader::sh_mesh       g_ShMesh;
AutoBindShader::sh_raster     g_ShRaster;
AutoBindShader::sh_implicit   g_ShImplicit;
AutoBindShader::sh_init       g_ShInit;

// --------------------------------------------------------------

using namespace LibSL::Mesh;
using namespace std;

// --------------------------------------------------------------

#define SCREEN_W   1024 // screen width and height
#define SCREEN_H   768

// view parameters
#define FOV      float(M_PI/4.0)
#define ZFAR     100.0f
#define ZNEAR    0.01f

#define CONNECT_NUM_ENTRIES 16 // max number of neighbors per centroids

int  g_NumCentroids = 111;      // initial number of centroids
int  g_GridSize     = -1;       // set by initialize()
int  g_NumTris      = -1;       // set by initialize()
bool g_Select       = true;     // selects mesh or implicit
bool g_MeshLoaded   = false;    // an input mesh was loaded
int  g_RasterRes    = 1024;     // resolution for off-screen rasterization

// --------------------------------------------------------------

// mesh to be remeshed and its LibSL renderer
TriangleMesh_Ptr                                       g_Mesh;
AutoPtr<MeshRenderer<MeshFormat_stl::t_VertexFormat> > g_Renderer;

// for creating a simple triangle, instanced for rendering result
typedef GPUMESH_MVF1(mvf_vertex_3f)                    mvf_mesh;
typedef GPUMesh_VertexBuffer<mvf_mesh>                 SimpleMesh;
AutoPtr<SimpleMesh>                                    g_GPUMesh_tri;

bool g_Iterate       = true; // controls whether we iterate
bool g_ShowMesh      = true; // controls whether the result is shown

/// GPU buffers
GLTexBuffer g_Centroids;     // centroid positions
GLTexBuffer g_Grid;          // grid (stores an index in GridData and a number of entries)
GLTexBuffer g_GridData;      // positions (x,y,z) and id for centroids stored in the grid
                             // for each grid entry, stores the list of centroids position and ids
                             // stores positions alongside id to avoid indirect lookups in Centroids
GLTexBuffer g_Counter;       // counts the number of triangles
GLTexBuffer g_AccumCenter;   // accumulates surface point positions per centroid
GLTexBuffer g_Connect;       // array of neighbords per centroid
GLTexBuffer g_Triangles;     // triangles (triplets of indices in Centroids)

RenderTarget2DLum_Ptr g_RT;

// --------------------------------------------------------------

// clears a buffer (zeroes)
// pixel shader: sh_clear
void clear(GLTexBuffer& buf)
{
  uint sz = (uint)ceil(sqrt((float)buf.size())) + 1;

  if (g_RT->w() < sz) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(sz, sz));
  }
  g_RT->bind();
  glViewport(0, 0, sz, sz);
  glDisable(GL_DEPTH_TEST);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShClear.begin();
  g_ShClear.u_ViewProj.set(vp);
  g_ShClear.u_ScreenWidth.set((int)sz);
  g_ShClear.u_Table.set(0);
  glBindImageTexture(0, buf.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShClear.u_TableSize.set(int(buf.size() / sizeof(uint)));
  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  g_ShClear.end();
  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

/* -------------------------------------------------------- */

// computes the area of a mesh triangle (for initial random sampling)
float triangleArea(int t)
{
  v3u tri = g_Mesh->triangleAt(t);
  v3f pts[3] = {
    g_Mesh->posAt(tri[0]),g_Mesh->posAt(tri[1]),g_Mesh->posAt(tri[2])
  };
  return length(cross(pts[1]-pts[0],pts[2]-pts[0]))*0.5f;
}

/* -------------------------------------------------------- */

// forward declarations
void raster(bool init);

/* -------------------------------------------------------- */

// places centroids along the mesh surface, at random
void randomize()
{
  clear(g_Grid);
  clear(g_Centroids);
  clear(g_Counter);

  raster(true);

  uint sz = (uint)ceil(sqrt((float)g_Grid.size())) + 1;
  if (g_RT->w() < sz) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(sz, sz));
  }
  g_RT->bind();

  glViewport(0, 0, sz, sz);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShInit.begin();
  g_ShInit.u_ViewProj.set(vp);
  g_ShInit.u_ScreenWidth.set((int)sz);

  g_ShInit.u_Counter.set(2);
  glBindImageTexture(2, g_Counter.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_ShInit.u_Centroids.set(1);
  glBindImageTexture(1, g_Centroids.glTexId(), 0, false, 0, GL_WRITE_ONLY, GL_R32F);
  g_ShInit.u_NumCentroids.set(g_NumCentroids);

  g_ShInit.u_Grid.set(0);
  glBindImageTexture(0, g_Grid.glTexId(), 0, false, 0, GL_READ_ONLY, GL_R32UI);
  g_ShInit.u_GridSize.set(v3i(g_GridSize, g_GridSize, g_GridSize));

  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  
  g_ShInit.end();

  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(2, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);

#if 0
  int numctrs = 0;
  {
    // read triangle count
    glBindBufferARB(GL_TEXTURE_BUFFER, g_Counter.glId());
    int *ptr = (int*)glMapBufferARB(GL_TEXTURE_BUFFER, GL_READ_ONLY);
    memcpy(&numctrs, ptr, sizeof(int));
    glUnmapBufferARB(GL_TEXTURE_BUFFER);
    glBindBufferARB(GL_TEXTURE_BUFFER, 0);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
  }
  cerr << "numctrs = " << numctrs << endl;
#endif
}

// --------------------------------------------------------------

// initializes all buffers, based on the current value of g_NumCentroids
void initialize()
{
  g_GridSize = (int)ceil(sqrt((float)g_NumCentroids));
  g_NumTris = g_NumCentroids * CONNECT_NUM_ENTRIES * 3;

  g_Grid.terminate();
  g_GridData.terminate();
  g_Centroids.terminate();
  g_Counter.terminate();
  g_AccumCenter.terminate();
  g_Connect.terminate();
  g_Triangles.terminate();

  g_Grid.init(g_GridSize * g_GridSize * g_GridSize * sizeof(uint) * 4 /*totcount,count,ptr | x,y,z,num */);
  g_GridData.init(g_NumCentroids * sizeof(float) * 4 /*x,y,z,id*/);
  g_Centroids.init(g_NumCentroids * sizeof(float) * 3 /*x,y,z*/);
  g_Counter.init(sizeof(uint));
  g_AccumCenter.init(g_NumCentroids * sizeof(float) * 4 /*x,y,z,coutn*/);
  g_Connect.init(g_NumCentroids * sizeof(uint) * CONNECT_NUM_ENTRIES);
  g_Triangles.init(g_NumTris * 3 * sizeof(uint));

  randomize();

  clear(g_Grid);
  clear(g_Counter);
  clear(g_AccumCenter);
  clear(g_Connect);

  LIBSL_GL_CHECK_ERROR;
}

// --------------------------------------------------------------

// counts the number of centroid enclosed in each of the grid entries
// pixel shader: sh_count
void count()
{
  uint sz = (uint)ceil(sqrt((float)g_NumCentroids)) + 1;

  if (g_RT->w() < sz) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(sz, sz));
  }

  g_RT->bind();

  glViewport(0, 0, sz, sz);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShCount.begin();
  g_ShCount.u_ViewProj.set(vp);
  g_ShCount.u_ScreenWidth.set((int)sz);

  g_ShCount.u_Centroids.set(1);
  glBindImageTexture(1, g_Centroids.glTexId(), 0, false, 0, GL_READ_ONLY, GL_R32F);
  g_ShCount.u_NumCentroids.set(g_NumCentroids);

  g_ShCount.u_Grid.set(0);
  glBindImageTexture(0, g_Grid.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShCount.u_GridSize.set(v3i(g_GridSize, g_GridSize, g_GridSize));

  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  g_ShCount.end();
  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

// --------------------------------------------------------------

// update the centroid positions, computing the average from the accumulated coords
// pixel shader: sh_update
void update()
{
  uint sz = (uint)ceil(sqrt((float)g_NumCentroids)) + 1;

  if (g_RT->w() < sz) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(sz, sz));
  }

  g_RT->bind();

  glViewport(0, 0, sz, sz);
  glDisable(GL_DEPTH_TEST);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShUpdate.begin();
  g_ShUpdate.u_ViewProj.set(vp);
  g_ShUpdate.u_ScreenWidth.set((int)sz);

  g_ShUpdate.u_Centroids.set(1);
  glBindImageTexture(1, g_Centroids.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32F);

  g_ShUpdate.u_AccumCenter.set(2);
  glBindImageTexture(2, g_AccumCenter.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_ShUpdate.u_NumCentroids.set(g_NumCentroids);

  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  g_ShCount.end();
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(2, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

// --------------------------------------------------------------

// computes start pointers for per-grid entry lists (after count)
// pixel shader: sh_allocate
void allocate()
{

  uint sz = (uint)ceil(sqrt((float)g_GridSize * g_GridSize * g_GridSize)) + 1;

  if (g_RT->w() < sz) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(sz, sz));
  }

  g_RT->bind();

  glViewport(0, 0, sz, sz);
  glDisable(GL_DEPTH_TEST);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShAllocate.begin();
  g_ShAllocate.u_ViewProj.set(vp);
  g_ShAllocate.u_ScreenWidth.set((int)sz);

  g_ShAllocate.u_Grid.set(1);
  glBindImageTexture(1, g_Grid.glTexId(), 0, false, 0, GL_READ_ONLY, GL_R32UI);
  g_ShAllocate.u_GridSize.set(g_GridSize * g_GridSize * g_GridSize);

  g_ShAllocate.u_Counter.set(0);
  glBindImageTexture(0, g_Counter.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);

  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  g_ShAllocate.end();

  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

// --------------------------------------------------------------

// stores the centroids into the per-grid entry list (after allocate)
// pixel shader: sh_store
void store()
{
  uint sz = (uint)ceil(sqrt((float)g_NumCentroids)) + 1;

  if (g_RT->w() < sz) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(sz, sz));
  }

  g_RT->bind();

  glViewport(0, 0, sz, sz);
  glDisable(GL_DEPTH_TEST);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShStore.begin();
  g_ShStore.u_ViewProj.set(vp);
  g_ShStore.u_ScreenWidth.set((int)sz);

  g_ShStore.u_Centroids.set(1);
  glBindImageTexture(1, g_Centroids.glTexId(), 0, false, 0, GL_READ_ONLY, GL_R32F);
  g_ShStore.u_NumCentroids.set(g_NumCentroids);

  g_ShStore.u_Grid.set(0);
  glBindImageTexture(0, g_Grid.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShStore.u_GridSize.set(v3i(g_GridSize, g_GridSize, g_GridSize));

  g_ShStore.u_GridData.set(2);
  glBindImageTexture(2, g_GridData.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32F);

  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  g_ShStore.end();

  glBindImageTexture(2, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

// --------------------------------------------------------------

// rasterizes the input mesh to be remeshed, accumulate coordinates for
// centroid position update (Lloyd), detects neighboring cells in
// Voronoi diagram
// pixel shader: sh_raster
void raster_mesh(bool init)
{
  uint res = g_RasterRes;
  if (g_RT->w() < res) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(res, res));
  }

  g_RT->bind();

  glViewport(0, 0, res, res);
  LibSL::GPUHelpers::clearScreen(LIBSL_COLOR_BUFFER | LIBSL_DEPTH_BUFFER, 0, 0, 1);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  g_ShRaster.begin();
  g_ShRaster.u_AcceptAll.set(0);
  g_ShRaster.u_Initializing.set(init ? 1 : 0);

  g_ShRaster.u_GridSize.set(v3i(g_GridSize, g_GridSize, g_GridSize));
  g_ShRaster.u_Grid.set(0);
  glBindImageTexture(0, g_Grid.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShRaster.u_GridData.set(1);
  glBindImageTexture(1, g_GridData.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32F);
  g_ShRaster.u_AccumCenter.set(2);
  glBindImageTexture(2, g_AccumCenter.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShRaster.u_Connect.set(4);
  glBindImageTexture(4, g_Connect.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShRaster.u_ConnectNumEntries.set(int(CONNECT_NUM_ENTRIES));

  // setup views and render
  m4x4f mproj = orthoMatrixGL<float>(-0.5f, 0.5f, -0.5f, 0.5f, -0.5f, 0.5f);
  g_ShRaster.u_ViewProj.set(mproj);
  g_ShRaster.u_Model.set(m4x4f::identity());
  g_Renderer->render();
  g_ShRaster.u_Model.set(quatf(v3f(1, 0, 0), float(M_PI) / 2.0f).toMatrix());
  g_Renderer->render();
  g_ShRaster.u_Model.set(quatf(v3f(0, 1, 0), float(M_PI) / 2.0f).toMatrix());
  g_Renderer->render();

  g_ShRaster.end();

  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(2, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(3, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(4, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

// --------------------------------------------------------------

// rasterizes the implicit surface be meshed, accumulate coordinates for
// centroid position update (Lloyd), detects neighboring cells in
// Voronoi diagram
// pixel shader: sh_implicit
void raster_implicit(bool init)
{
  uint res = g_RasterRes;
  if (g_RT->w() < res) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(res, res));
  }

  g_RT->bind();

  glViewport(0, 0, res, res);
  LibSL::GPUHelpers::clearScreen(LIBSL_COLOR_BUFFER | LIBSL_DEPTH_BUFFER, 0, 0, 1);

  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShImplicit.begin();
  g_ShImplicit.u_Initializing.set(init ? 1 : 0);
  g_ShImplicit.u_GridSize.set(v3i(g_GridSize, g_GridSize, g_GridSize));
  g_ShImplicit.u_Grid.set(0);
  glBindImageTexture(0, g_Grid.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShImplicit.u_GridData.set(1);
  glBindImageTexture(1, g_GridData.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32F);
  g_ShImplicit.u_AccumCenter.set(2);
  glBindImageTexture(2, g_AccumCenter.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShImplicit.u_Connect.set(4);
  glBindImageTexture(4, g_Connect.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShImplicit.u_ConnectNumEntries.set(int(CONNECT_NUM_ENTRIES));
  g_ShImplicit.u_ViewProj.set(vp);
  g_ShImplicit.iResolution.set(v2f(res, res));
  static float time = 0.0f;
  g_ShImplicit.iTime.set(time);
  time += 0.005f;

  g_ShImplicit.u_Dir.set(0);
  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  
  g_ShImplicit.u_Dir.set(1);
  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();

  g_ShImplicit.u_Dir.set(2);
  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();

  g_ShImplicit.end();
  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(2, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(3, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(4, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

// --------------------------------------------------------------

void raster(bool init)
{
  if (g_Select) {
    raster_implicit(init);
  } else {
    raster_mesh(init);
  }
}

// --------------------------------------------------------------

// generate triangles given all other information
// pixel shader: sh_gentris
void gentris()
{
  uint sz = (uint)ceil(sqrt((float)g_NumCentroids)) + 1;

  if (g_RT->w() < sz) {
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(sz, sz));
  }

  g_RT->bind();

  glViewport(0, 0, sz, sz);
  glDisable(GL_DEPTH_TEST);

  m4x4f vp = orthoMatrixGL<float>(0, 1, 0, 1, -1, 1);

  g_ShGenTris.begin();
  g_ShGenTris.u_ViewProj.set(vp);
  g_ShGenTris.u_ScreenWidth.set(uint(sz));
  g_ShGenTris.u_NumCentroids.set(g_NumCentroids);

  g_ShGenTris.u_Triangles.set(0);
  glBindImageTexture(0, g_Triangles.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_ShGenTris.u_Connect.set(2);
  glBindImageTexture(2, g_Connect.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShGenTris.u_ConnectNumEntries.set(uint(CONNECT_NUM_ENTRIES));

  g_ShGenTris.u_Counter.set(3);
  glBindImageTexture(3, g_Counter.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);

  glBegin(GL_QUADS);
  glVertex2i(0, 0); glVertex2i(1, 0); glVertex2i(1, 1); glVertex2i(0, 1);
  glEnd();
  g_ShCount.end();
  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(2, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(3, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  g_RT->unbind();

  glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

// --------------------------------------------------------------

// on-screen display of the final result
// pixel shader: sh_mesh (result) / sh_raster (Voronoi)
void display()
{
  /// render on screen
  LibSL::GPUHelpers::clearScreen(LIBSL_COLOR_BUFFER | LIBSL_DEPTH_BUFFER, 0.3f, 0.3f, 0.3f);

  glViewport(0, 0, SCREEN_W, SCREEN_H);

  // setup view
  m4x4f mproj = perspectiveMatrix(FOV, SCREEN_W / float(SCREEN_H), ZNEAR, ZFAR);

  glEnable(GL_DEPTH_TEST);

  glMemoryBarrier(GL_ALL_BARRIER_BITS);

  static Array<v4f> centroids(g_NumCentroids);
  static Array<int> connect(g_NumCentroids * CONNECT_NUM_ENTRIES);
  static Array<int> tris(g_NumTris * 3);
  if (centroids.size() != g_NumCentroids) {
    centroids.allocate(g_NumCentroids);
    connect.allocate(g_NumCentroids * CONNECT_NUM_ENTRIES);
    tris.allocate(g_NumTris * 3);
  }

  // draw mesh
  int numtris = 0;
  {
    // read triangle count
    glBindBufferARB(GL_TEXTURE_BUFFER, g_Counter.glId());
    int *ptr = (int*)glMapBufferARB(GL_TEXTURE_BUFFER, GL_READ_ONLY);
    memcpy(&numtris, ptr, sizeof(int));
    glUnmapBufferARB(GL_TEXTURE_BUFFER);
    glBindBufferARB(GL_TEXTURE_BUFFER, 0);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
  }
  Transform::set(LIBSL_PROJECTION_MATRIX, mproj);
  Transform::set(LIBSL_MODELVIEW_MATRIX, TrackballUI::matrix());
  g_ShMesh.begin();
  g_ShMesh.u_Centroids.set(1);
  glBindImageTexture(1, g_Centroids.glTexId(), 0, false, 0, GL_READ_ONLY, GL_R32F);
  g_ShMesh.u_Triangles.set(2);
  glBindImageTexture(2, g_Triangles.glTexId(), 0, false, 0, GL_READ_WRITE, GL_R32UI);
  g_ShMesh.u_Proj.set(mproj);
  g_ShMesh.u_Model.set(TrackballUI::matrix());
  g_GPUMesh_tri->instantiate(numtris);
  g_ShMesh.end();
  glBindImageTexture(0, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(1, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);
  glBindImageTexture(2, 0, 0, false, 0, GL_READ_WRITE, GL_R32UI);

  // raster_implicit(false);
}

// --------------------------------------------------------------

// produces a mesh for saving it with LibSL
TriangleMesh_Ptr makeMesh()
{
  Array<v3f> centroids(g_NumCentroids);
  Array<int> connect(g_NumCentroids * CONNECT_NUM_ENTRIES);
  Array<int> tris(g_NumTris * 3);
  int counter = 0;
  {
    glBindBufferARB(GL_TEXTURE_BUFFER, g_Centroids.glId());
    v3f *ptr = (v3f*)glMapBufferARB(GL_TEXTURE_BUFFER, GL_READ_ONLY);
    memcpy(&centroids[0][0], ptr, centroids.size() * sizeof(v3f));
    glUnmapBufferARB(GL_TEXTURE_BUFFER);
    glBindBufferARB(GL_TEXTURE_BUFFER, 0);
  }
  {
    glBindBufferARB(GL_TEXTURE_BUFFER, g_Connect.glId());
    int *ptr = (int*)glMapBufferARB(GL_TEXTURE_BUFFER, GL_READ_ONLY);
    memcpy(connect.raw(), ptr, g_NumCentroids * CONNECT_NUM_ENTRIES * sizeof(int));
    glUnmapBufferARB(GL_TEXTURE_BUFFER);
    glBindBufferARB(GL_TEXTURE_BUFFER, 0);
  }
  {
    glBindBufferARB(GL_TEXTURE_BUFFER, g_Triangles.glId());
    int *ptr = (int*)glMapBufferARB(GL_TEXTURE_BUFFER, GL_READ_ONLY);
    memcpy(tris.raw(), ptr, g_NumTris * 3 * sizeof(int));
    glUnmapBufferARB(GL_TEXTURE_BUFFER);
    glBindBufferARB(GL_TEXTURE_BUFFER, 0);
  }
  {
    glBindBufferARB(GL_TEXTURE_BUFFER, g_Counter.glId());
    int *ptr = (int*)glMapBufferARB(GL_TEXTURE_BUFFER, GL_READ_ONLY);
    memcpy(&counter, ptr, sizeof(int));
    glUnmapBufferARB(GL_TEXTURE_BUFFER);
    glBindBufferARB(GL_TEXTURE_BUFFER, 0);
  }
  glMemoryBarrier(GL_ALL_BARRIER_BITS);

  // triangles
  TriangleMesh_Ptr mesh(new TriangleMesh_generic<MeshFormat_stl::t_VertexData>(counter*3,counter));
  ForIndex(t, counter) {
    ForIndex(i, 3) {
      int id = tris[t * 3 + i];
      v3f p  = v3f(centroids[id]) * 2.0f - v3f(1.0f);
      mesh->posAt(t*3+i) = p;
    }
    mesh->triangleAt(t) = v3u(t * 3, t * 3 + 1, t * 3 + 2);
  }
  mesh->mergeVerticesExact();
  mesh->reorientTriangles();
  return mesh;
}

// --------------------------------------------------------------

// iterates the remeshing process
void iter(bool upd = false)
{
  clear(g_Grid);
  clear(g_Counter);
  clear(g_AccumCenter);
  clear(g_Connect);
  count();
  allocate();
  store();
  raster(false);
  clear(g_Counter);
  gentris();
  if (upd) {
    update();
  }
}

/* -------------------------------------------------------- */

// keyboard events
void mainKeyboard(unsigned char key)
{

  if (key == ' ') {
    g_Iterate = !g_Iterate;
  } else if (key == 'm') {
    g_ShowMesh = !g_ShowMesh;
  } else if (key == 'r') {
    randomize();
  } else if (key == 's') {
    // save mesh
    auto m = makeMesh();
    saveTriangleMesh("out.stl", m.raw());
  } else if (key == 'q') {
    TrackballUI::exit();
  }

}

// --------------------------------------------------------------

// main render loop
void mainRender()
{
  if (g_Iterate) {
    iter(true);
  }

  display();

  ImGui::SetNextWindowSize(ImVec2(350, 100),ImGuiCond_Once);

  ImGui::Begin("Raster2Mesh (origins)");
  ImGui::Text("<< Coarser | Finer >>");
  static int numc = g_NumCentroids;
  ImGui::SliderInt("##slider", &numc, 32, 1024);
  if (numc != g_NumCentroids) {
    g_NumCentroids = numc;
    initialize();
  }
  if (g_MeshLoaded) {
    if (ImGui::Checkbox("Input mesh or implicit", &g_Select)) {
      g_RasterRes = g_Select ? 1024 : 512;
      initialize();
    }
  }
  ImGui::End();

  ImGui::Render();
}

/* -------------------------------------------------------- */

int main(int argc, char **argv)
{
  try {

    /// init simple UI (glut clone for both GL and D3D)
    cerr << "Init TrackballUI   ";
    TrackballUI::onRender = mainRender;
    TrackballUI::onKeyPressed = mainKeyboard;
    TrackballUI::init(SCREEN_W, SCREEN_H);
    cerr << "[OK]" << endl;

    SimpleUI::bindImGui();

    /// help
    printf("[ESC]    - quit\n");

    // trackball
    TrackballUI::setCenter(V3F(0, 0, 0));
    TrackballUI::trackball().rotation()    = quatf(v3f(1, 0, 0), -(float)M_PI/2.0f );
    TrackballUI::trackball().translation() = v3f(0, 0, -1.44f);

    /// load mesh
    if (argc > 1) {
      g_Mesh = TriangleMesh_Ptr(loadTriangleMesh<MeshFormat_stl::t_VertexData,
        MeshFormat_stl::t_VertexFormat>(argv[1]));
      g_Mesh->scaleToUnitCube(0.95f);
      g_Mesh->centerOn(v3f(0));
      g_Renderer = AutoPtr<MeshRenderer<MeshFormat_stl::t_VertexFormat> >(new MeshRenderer<MeshFormat_stl::t_VertexFormat>(g_Mesh.raw()));
      g_MeshLoaded = true;
    }

    /// GPU shaders init
    g_ShRaster.init(GL_TRIANGLES, GL_TRIANGLE_STRIP, 3);
    g_ShMesh.init();
    g_ShClear.init();
    g_ShCount.init();
    g_ShInit.init();
    g_ShAllocate.init();
    g_ShStore.init();
    g_ShUpdate.init();
    g_ShGenTris.init();
    g_ShImplicit.init();

    g_ShRaster.begin();
    g_ShRaster.u_ComputeConnectivity.set(1);
    g_ShRaster.u_Initializing.set(0);
    g_ShRaster.end();
    g_ShImplicit.begin();
    g_ShImplicit.u_ComputeConnectivity.set(1);
    g_ShImplicit.u_Initializing.set(0);
    g_ShImplicit.end();
    /// render target for off-screen rasterization
    g_RT = RenderTarget2DLum_Ptr(new RenderTarget2DLum(g_RasterRes, g_RasterRes));
    /// triangle for instantiation
    g_GPUMesh_tri = AutoPtr<SimpleMesh>(new SimpleMesh());
    g_GPUMesh_tri->begin(GPUMESH_TRIANGLELIST);
    g_GPUMesh_tri->vertex_3(0, 0, 0);
    g_GPUMesh_tri->vertex_3(1, 0, 0);
    g_GPUMesh_tri->vertex_3(0, 1, 0);
    g_GPUMesh_tri->end();
    /// initialize
    initialize();
    /// bind imgui
    SimpleUI::initImGui();
    SimpleUI::onReshape(SCREEN_W, SCREEN_H);

    /// main loop
    TrackballUI::loop();

    /// clean exit
    g_ShImplicit.terminate();
    g_ShRaster.terminate();
    g_ShMesh.terminate();
    g_ShClear.terminate();
    g_ShCount.terminate();
    g_ShInit.terminate();
    g_ShAllocate.terminate();
    g_ShStore.terminate();
    g_ShUpdate.terminate();
    g_ShGenTris.terminate();
    g_Connect.terminate();
    g_Counter.terminate();
    g_AccumCenter.terminate();
    g_Grid.terminate();
    g_GridData.terminate();
    g_Centroids.terminate();
    g_Triangles.terminate();
    g_GPUMesh_tri = AutoPtr<SimpleMesh>();
    /// shutdown TrackballUI
    TrackballUI::shutdown();

  } catch (Fatal& e) {
    cerr << e.message() << endl;
    return (-1);
  }

  return (0);
}

/* -------------------------------------------------------- */

LIBSL_WIN32_FIX; // globally registers file loaders (meshes, images)

/* -------------------------------------------------------- */
