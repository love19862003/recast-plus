//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "Sample.h"
#include "Sample_TileMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "NavMeshPruneTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "SampleInterfaces.h"

using namespace NavSpace;
#ifdef WIN32
#	define snprintf _snprintf
#endif

// 
// inline unsigned int nextPow2(unsigned int v)
// {
// 	v--;
// 	v |= v >> 1;
// 	v |= v >> 2;
// 	v |= v >> 4;
// 	v |= v >> 8;
// 	v |= v >> 16;
// 	v++;
// 	return v;
// }
// 
// inline unsigned int ilog2(unsigned int v)
// {
// 	unsigned int r;
// 	unsigned int shift;
// 	r = (v > 0xffff) << 4; v >>= r;
// 	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
// 	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
// 	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
// 	r |= (v >> 1);
// 	return r;
// }

class NavMeshTileTool : public SampleTool
{
	Sample_TileMesh* m_sample;
	float m_hitPos[3];
	bool m_hitPosSet;
  bool m_recalculate = false;
  bool m_addMesh = false;
  float m_scale = 1.f;
  float m_dir = 0.f;
  unsigned int m_addId = INVALID_MESH_ID;
public:

	NavMeshTileTool() :
		m_sample(0),
		m_hitPosSet(false)
	{
		m_hitPos[0] = m_hitPos[1] = m_hitPos[2] = 0;
	}

	virtual ~NavMeshTileTool()
	{
	}

	virtual int type() { return TOOL_TILE_EDIT; }

	virtual void init(Sample* sample)
	{
		m_sample = (Sample_TileMesh*)sample; 
	}
	
	virtual void reset() {}

	virtual void handleMenu()
	{
		imguiLabel("Create Tiles");
		if (imguiButton("Create All"))
		{
			if (m_sample)
				m_sample->buildAllTiles();
		}
		if (imguiButton("Remove All"))
		{
			if (m_sample)
				m_sample->removeAll();
		}

    if (imguiCheck("calculate bonus", m_recalculate)){
      m_recalculate = !m_recalculate;
    }

    if(imguiCheck("action object", m_addMesh)){
      m_addMesh = !m_addMesh;
    }

    if (m_addMesh){
      imguiSlider("scale", &m_scale, 0.1f, 1.f, 0.1f);
      imguiSlider("dir", &m_dir, 0, 360, 10);

      if (m_sample){
        auto& meshs = m_sample->meshMap().constRefMap();
        for (auto& pair : meshs){
          if (imguiCheck(("id:" + std::to_string(pair.first) + " path:" + pair.second->name()).data(), m_addId == pair.first)){
            m_addId = pair.first;
          }
        }
      }
    }

	}

	virtual void handleClick(const float* s, const float* p, bool shift)
	{
		m_hitPosSet = true;
		rcVcopy(m_hitPos,p);
		if (m_sample)
		{
      if (m_addMesh){
        if (shift)
          m_sample->removeObject(s, p, m_recalculate);
        else
          m_sample->addObject(m_hitPos, m_scale, m_dir, m_addId, m_recalculate);
      }else{
        if (shift)
          m_sample->removeTile(m_hitPos);
        else
          m_sample->buildTileWithPos(m_hitPos);
      }
			
		}
	}

	virtual void handleToggle() {}

	virtual void handleStep() {}

	virtual void handleUpdate(const float /*dt*/) {}
	
	virtual void handleRender()
	{
		if (m_hitPosSet)
		{
			const float s = m_sample->getAgentRadius();
			glColor4ub(0,0,0,128);
			glLineWidth(2.0f);
			glBegin(GL_LINES);
			glVertex3f(m_hitPos[0]-s,m_hitPos[1]+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0]+s,m_hitPos[1]+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]-s+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]+s+0.1f,m_hitPos[2]);
			glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]-s);
			glVertex3f(m_hitPos[0],m_hitPos[1]+0.1f,m_hitPos[2]+s);
			glEnd();
			glLineWidth(1.0f);
		}
	}
	
	virtual void handleRenderOverlay(double* proj, double* model, int* view)
	{
		GLdouble x, y, z;
		if (m_hitPosSet && gluProject((GLdouble)m_hitPos[0], (GLdouble)m_hitPos[1], (GLdouble)m_hitPos[2],
									  model, proj, view, &x, &y, &z))
		{
			int tx=0, ty=0;
			m_sample->getTilePos(m_hitPos, tx, ty);
			char text[32];
			snprintf(text,32,"(%d,%d)", tx,ty);
			imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
		}
		
		// Tool help
		const int h = view[3];
		imguiDrawText(280, h-40, IMGUI_ALIGN_LEFT, "LMB: Rebuild hit tile.  Shift+LMB: Clear hit tile.", imguiRGBA(255,255,255,192));	
	}
};




Sample_TileMesh::Sample_TileMesh() :
	m_keepInterResults(false),
	m_buildAll(true),
	m_totalBuildTimeMs(0),
	m_drawMode(DRAWMODE_NAVMESH),
	m_tileCol(duRGBA(0,0,0,32)),
	m_tileBuildTime(0),
	m_tileMemUsage(0),
	m_tileTriCount(0)
{
	memset(m_lastBuiltTileBmin, 0, sizeof(m_lastBuiltTileBmin));
	memset(m_lastBuiltTileBmax, 0, sizeof(m_lastBuiltTileBmax));
	
	setTool(new NavMeshTileTool);
}

Sample_TileMesh::~Sample_TileMesh()
{
	
}


void Sample_TileMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	if (imguiCheck("Build All Tiles", m_buildAll))
		m_buildAll = !m_buildAll;
	
	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_setting.tileSize, 8.0f, 1024.0f, 8.0f);
	
  bool enableBuild = false;
  if (hasScene()){
		char text[64];
		int gw = 0, gh = 0;
		const float* bmin = m_setting.navBmin;
		const float* bmax = m_setting.navBmax;
		rcCalcGridSize(bmin, bmax, m_setting.cellSize, &gw, &gh);
		const int ts = (int)m_setting.tileSize;
		const int tw = (gw + ts-1) / ts;
		const int th = (gh + ts-1) / ts;
		snprintf(text, 64, "Tiles  %d x %d", tw, th);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
// 		int tileBits = std::min<int>(tileBit(tw*th), 14);
// 		if (tileBits > 14) tileBits = 14;
// 		int polyBits = 22 - tileBits;
    int tileBits = tileBit(tw*th);
    int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
		snprintf(text, 64, "Max Tiles  %d", m_maxTiles);
		imguiValue(text);
		snprintf(text, 64, "Max Polys  %d", m_maxPolysPerTile);
		imguiValue(text);
    enableBuild = tw * th <= m_maxTiles;
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}
	
	imguiSeparator();
	
	imguiIndent();
	imguiIndent();
	
	if (imguiButton("SaveMapBin(F5)"))
	{
    saveMapBin();
	}

  if (imguiButton("SaveNavMesh(F2)")){
    saveMapNavMesh();
  }

	if (imguiButton("LoadNavMesh(F3)"))
	{
    loadMapNavMesh();
	}


  if (imguiButton("SaveDumpScene(F4)")){
    saveDumpScene();
  }
	imguiUnindent();
	imguiUnindent();
	
	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);
	
	imguiSeparator();
	
	imguiSeparator();
	


  if (imguiButton("Build(F1)", enableBuild)){
    handleBuild();
  }

}

void Sample_TileMesh::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Prune Navmesh", type == TOOL_NAVMESH_PRUNE))
	{
		setTool(new NavMeshPruneTool);
	}
	if (imguiCheck("Create Tiles", type == TOOL_TILE_EDIT))
	{
		setTool(new NavMeshTileTool);
	}
	if (imguiCheck("Create Off-Mesh Links", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}
	
	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();
}

void Sample_TileMesh::handleDebugMode()
{
	// Check which modes are valid.
	bool valid[MAX_DRAWMODE];
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		valid[i] = false;
	
// 	if (m_geom)
// 	{
		valid[DRAWMODE_NAVMESH] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_PORTALS] = m_navMesh != 0;
		valid[DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valid[DRAWMODE_MESH] = true;
		valid[DRAWMODE_VOXELS] = m_solid != 0;
		valid[DRAWMODE_VOXELS_WALKABLE] = m_solid != 0;
		valid[DRAWMODE_COMPACT] = m_chf != 0;
		valid[DRAWMODE_COMPACT_DISTANCE] = m_chf != 0;
		valid[DRAWMODE_COMPACT_REGIONS] = m_chf != 0;
		valid[DRAWMODE_REGION_CONNECTIONS] = m_cset != 0;
		valid[DRAWMODE_RAW_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_BOTH_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_CONTOURS] = m_cset != 0;
		valid[DRAWMODE_POLYMESH] = m_pmesh != 0;
		valid[DRAWMODE_POLYMESH_DETAIL] = m_dmesh != 0;
//	}
	
	int unavail = 0;
	for (int i = 0; i < MAX_DRAWMODE; ++i)
		if (!valid[i]) unavail++;
	
	if (unavail == MAX_DRAWMODE)
		return;
	
	imguiLabel("Draw");
  if (imguiCheck("TreeBox", m_drawBoxTree)){ m_drawBoxTree = !m_drawBoxTree; }
	if (imguiCheck("Input Mesh", m_drawMode == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;
	if (imguiCheck("Navmesh", m_drawMode == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;
	if (imguiCheck("Navmesh Invis", m_drawMode == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;
	if (imguiCheck("Navmesh Trans", m_drawMode == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;
	if (imguiCheck("Navmesh BVTree", m_drawMode == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;
	if (imguiCheck("Navmesh Nodes", m_drawMode == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;
	if (imguiCheck("Navmesh Portals", m_drawMode == DRAWMODE_NAVMESH_PORTALS, valid[DRAWMODE_NAVMESH_PORTALS]))
		m_drawMode = DRAWMODE_NAVMESH_PORTALS;
	if (imguiCheck("Voxels", m_drawMode == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
		m_drawMode = DRAWMODE_VOXELS;
	if (imguiCheck("Walkable Voxels", m_drawMode == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;
	if (imguiCheck("Compact", m_drawMode == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
		m_drawMode = DRAWMODE_COMPACT;
	if (imguiCheck("Compact Distance", m_drawMode == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;
	if (imguiCheck("Compact Regions", m_drawMode == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
		m_drawMode = DRAWMODE_COMPACT_REGIONS;
	if (imguiCheck("Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;
	if (imguiCheck("Raw Contours", m_drawMode == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
		m_drawMode = DRAWMODE_RAW_CONTOURS;
	if (imguiCheck("Both Contours", m_drawMode == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
		m_drawMode = DRAWMODE_BOTH_CONTOURS;
	if (imguiCheck("Contours", m_drawMode == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
		m_drawMode = DRAWMODE_CONTOURS;
	if (imguiCheck("Poly Mesh", m_drawMode == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
		m_drawMode = DRAWMODE_POLYMESH;
	if (imguiCheck("Poly Mesh Detail", m_drawMode == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
		m_drawMode = DRAWMODE_POLYMESH_DETAIL;
		
	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("rebuild some tiles to see");
		imguiValue("more debug mode options.");
	}
}

void Sample_TileMesh::handleRender()
{
	if (!hasScene())
		return;
	
	//const float texScale = 1.0f / (m_cellSize * 10.0f);
	
	// Draw mesh
	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
// 		duDebugDrawTriMeshSlope(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
// 								m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(),
// 								m_agentMaxSlope, texScale);
// 		m_geom->drawOffMeshConnections(&m_dd);

    drawMeshObjects(&m_dd);
    
	}
		
	glDepthMask(GL_FALSE);
	
	// Draw bounds
  const float* bmin = m_setting.navBmin;//m_geom->getNavMeshBoundsMin();
  const float* bmax = m_setting.navBmax;// m_geom->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duRGBA(255,255,255,128), 1.0f);
	
	// Tiling grid.
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_setting.cellSize, &gw, &gh);
	const int tw = (gw + (int)m_setting.tileSize-1) / (int)m_setting.tileSize;
	const int th = (gh + (int)m_setting.tileSize-1) / (int)m_setting.tileSize;
	const float s = m_setting.tileSize*m_setting.cellSize;
	duDebugDrawGridXZ(&m_dd, bmin[0],bmin[1],bmin[2], tw,th, s, duRGBA(0,0,0,64), 1.0f);
	
	// Draw active tile
	duDebugDrawBoxWire(&m_dd, m_lastBuiltTileBmin[0],m_lastBuiltTileBmin[1],m_lastBuiltTileBmin[2],
					   m_lastBuiltTileBmax[0],m_lastBuiltTileBmax[1],m_lastBuiltTileBmax[2], m_tileCol, 1.0f);
		
	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
		 m_drawMode == DRAWMODE_NAVMESH_TRANS ||
		 m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
		 m_drawMode == DRAWMODE_NAVMESH_NODES ||
		 m_drawMode == DRAWMODE_NAVMESH_PORTALS ||
		 m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&m_dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&m_dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_PORTALS)
			duDebugDrawNavMeshPortals(&m_dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&m_dd, *m_navQuery);
		duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0,0,0,128));
	}
	
	
	glDepthMask(GL_TRUE);
	
	if (m_chf && m_drawMode == DRAWMODE_COMPACT)
		duDebugDrawCompactHeightfieldSolid(&m_dd, *m_chf);
	
	if (m_chf && m_drawMode == DRAWMODE_COMPACT_DISTANCE)
		duDebugDrawCompactHeightfieldDistance(&m_dd, *m_chf);
	if (m_chf && m_drawMode == DRAWMODE_COMPACT_REGIONS)
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
	if (m_solid && m_drawMode == DRAWMODE_VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_solid && m_drawMode == DRAWMODE_VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	
	if (m_cset && m_drawMode == DRAWMODE_RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	
	if (m_cset && m_drawMode == DRAWMODE_BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset, 0.5f);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DRAWMODE_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_chf && m_cset && m_drawMode == DRAWMODE_REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
		
		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_pmesh && m_drawMode == DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_dd, *m_pmesh);
		glDepthMask(GL_TRUE);
	}
	if (m_dmesh && m_drawMode == DRAWMODE_POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&m_dd, *m_dmesh);
		glDepthMask(GL_TRUE);
	}
		
	//m_geom->drawConvexVolumes(&m_dd);
  drawVolumes(&m_dd);
 
  drawOffConPools(&m_dd);
	if (m_tool)
		m_tool->handleRender();
	renderToolStates();
 
	glDepthMask(GL_TRUE);
}

void Sample_TileMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	
	// Draw start and end point labels
	if (m_tileBuildTime > 0.0f && gluProject((GLdouble)(m_lastBuiltTileBmin[0]+m_lastBuiltTileBmax[0])/2, (GLdouble)(m_lastBuiltTileBmin[1]+m_lastBuiltTileBmax[1])/2, (GLdouble)(m_lastBuiltTileBmin[2]+m_lastBuiltTileBmax[2])/2,
											 model, proj, view, &x, &y, &z))
	{
		char text[32];
		snprintf(text,32,"%.3fms / %dTris / %.1fkB", m_tileBuildTime, m_tileTriCount, m_tileMemUsage);
		imguiDrawText((int)x, (int)y-25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0,0,0,220));
	}
	
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_TileMesh::handleMeshChanged(const std::string& file)
{
	Sample::handleMapChanged(file);

// 	const BuildSettings* buildSettings = geom->getBuildSettings();
// 	if (buildSettings && buildSettings->tileSize > 0)
// 		m_tileSize = buildSettings->tileSize;

	//cleanup();

// 	dtFreeNavMesh(m_navMesh);
// 	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_TileMesh::handleBuild()
{
  if (m_buildAll && !build())
    return false;
	
	if (m_tool)
		m_tool->init(this);
	initToolStates(this);

	return true;
}

void Sample_TileMesh::collectSettings(BuildSettings& settings)
{
	Sample::collectSettings(settings);

	//settings.tileSize = m_tileSize;
}

void Sample_TileMesh::moveToNextObject(float* cameraPos, float* cameraEulers){
  ++m_moveTarget;
  if (!m_objects.hasData(m_moveTarget)){
    m_moveTarget = INVALID_MOBJ_ID;
  }

  auto obj = m_objects.getData(m_moveTarget);
  if (!obj){
    return;
  }

  auto& item = obj->item();

  const float* bmin = obj->m_bouns.bmin.data();
  const float* bmax = obj->m_bouns.bmax.data();
  float camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
               rcSqr(bmax[1] - bmin[1]) +
               rcSqr(bmax[2] - bmin[2])) / 2;
  cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
  cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
  cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
  cameraEulers[0] = 45;
  cameraEulers[1] = -45;
}

void Sample_TileMesh::buildTileWithPos(const float* pos)
{
	
	if (!m_navMesh) return;
		
  const float* bmin = m_setting.navBmin;
  const float* bmax = m_setting.navBmin;

	
	const float ts = m_setting.tileSize* m_setting.cellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);
	
	m_lastBuiltTileBmin[0] = bmin[0] + tx*ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty*ts;
	
	m_lastBuiltTileBmax[0] = bmin[0] + (tx+1)*ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty+1)*ts;
	
	m_tileCol = duRGBA(255,255,255,64);
	
	m_ctx->resetLog();
	
  buildTile(std::make_pair(tx, ty));
	
	//m_ctx->dumpLog("Build Tile (%d,%d):", tx,ty);
}

void Sample_TileMesh::getTilePos(const float* pos, int& tx, int& ty)
{
	//if (!m_geom) return;
	
  const float* bmin = m_setting.navBmin;/* m_geom->getNavMeshBoundsMin();*/
	
	const float ts = m_setting.tileSize* m_setting.cellSize;
	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}

void Sample_TileMesh::removeTile(const float* pos)
{
	/*if (!m_geom) return;*/
	if (!m_navMesh) return;
	
  const float* bmin = m_setting.navBmin;
  const float* bmax = m_setting.navBmax;

  const float ts = m_setting.tileSize*m_setting.cellSize;
  const int tx = (int)((pos[0] - bmin[0]) / ts);
  const int ty = (int)((pos[2] - bmin[2]) / ts);

	m_lastBuiltTileBmin[0] = bmin[0] + tx*ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty*ts;
	
	m_lastBuiltTileBmax[0] = bmin[0] + (tx+1)*ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty+1)*ts;
	
	m_tileCol = duRGBA(128,32,16,64);
	
	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx,ty,0),0,0);
}

void Sample_TileMesh::buildAllTiles()
{
	if (!m_navMesh) return;
	
	const float* bmin = m_setting.navBmin;
	const float* bmax = m_setting.navBmax;
	int gw = 0, gh = 0;
	rcCalcGridSize(bmin, bmax, m_setting.cellSize, &gw, &gh);
	const int ts = (int)m_setting.tileSize;
	const int tw = (gw + ts-1) / ts;
	const int th = (gh + ts-1) / ts;
	const float tcs = m_setting.tileSize*m_setting.cellSize;

	
	// Start the build process.
	m_ctx->startTimer(RC_TIMER_TEMP);

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			m_lastBuiltTileBmin[0] = bmin[0] + x*tcs;
			m_lastBuiltTileBmin[1] = bmin[1];
			m_lastBuiltTileBmin[2] = bmin[2] + y*tcs;
			
			m_lastBuiltTileBmax[0] = bmin[0] + (x+1)*tcs;
			m_lastBuiltTileBmax[1] = bmax[1];
			m_lastBuiltTileBmax[2] = bmin[2] + (y+1)*tcs;
			
      buildTile(std::make_pair(x, y));
		}
	}
	
	// Start the build process.	
	m_ctx->stopTimer(RC_TIMER_TEMP);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TEMP)/1000.0f;
	
}