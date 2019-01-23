/************************************************
 * \file NavTool.h
 * \date 2019/01/16 16:29
 *
 * \author wufan
 * Contact: love19862003@163.com
 *
 * \brief for nav tool demo
 *
 * TODO: long description
 *
 * \note
*************************************************/
#pragma once
#include "NavManager.h"
#include "NavScene.h"
#include "Recast.h"
class dtCrowd;

namespace NavSpace{

  class NavTool : public NavScene, public NavManager, public NonCopyAble
  {
  public:
    explicit NavTool(rcContext* ctx);
    virtual ~NavTool();


    bool load(const std::string& file);

    virtual bool buildTile(const TileIndex& tile) final;

    bool build();

    void saveMapNavMesh();
    void saveMapBin();
    void saveMegerMapBin();
    void saveMeshBin(const MeshPtr& ptr);

    void loadMapNavMesh();
    void saveDumpScene();

    void addMesh(const std::string& file);

    void removeTile(const TileIndex& index);
    void removeAll();

    //virtual class InputGeom* getInputGeom(){ return m_geom; }
    virtual class dtNavMesh* getNavMesh(){ return m_navMesh; }
    virtual class dtNavMeshQuery* getNavMeshQuery(){ return m_navQuery; }
    virtual class dtCrowd* getCrowd(){ return m_crowd; }
    virtual float getAgentRadius(){ return m_setting.agentRadius; }
    virtual float getAgentHeight(){ return m_setting.agentHeight; }
    virtual float getAgentClimb(){ return m_setting.agentMaxClimb; }

    bool hasScene() const{ return m_objects.hasData(INVALID_MOBJ_ID); }
    
    TileIndex getTileIndex(const float* pos) const;

    bool reset();

    

    ObjectPtr getScene() const{ return m_objects.getData(INVALID_MOBJ_ID); }

    void addConvexVolume(const float* verts, const int nverts,
                         const float minh, const float maxh, unsigned char area);

    void deleteConvexVolume(int i);

    void addOffMeshConnection(const float* spos, const float* epos, const float rad,
                              unsigned char bidir, unsigned char area, unsigned short flags);
    void deleteOffMeshConnection(int i);

    void drawOffConPools(struct duDebugDraw* dd, bool hilight = false);

    void drawVolumes(struct duDebugDraw* dd, bool hilight = false);

    void drawMeshObjects(struct duDebugDraw* dd);

    bool raycastMesh(float* src, float* dst, float& tmin);

    MObjId hitTestMesh(const float* s, const float* e) ;

    bool setScenePtr(ObjectPtr ptr);

    int tileBit(int v) const;

    const MeshMap& meshMap() const{ return m_meshs; }

    void removeObject(const float* s, const float* e, bool rec);

    void addObject(const float* pos, float scale, float o, MeshId id, bool rec);

    void addObject(ObjectPtr ptr);

    bool megerObjects(const std::string& file);
  protected:
    void cleanup();
  protected:
    MeshId m_nextMeshId = INVALID_MESH_ID;
    MObjId m_nextObjId = INVALID_MOBJ_ID;
    bool m_drawBoxTree = false;
    dtCrowd* m_crowd;
    rcContext* m_ctx;
    rcConfig m_cfg;
    rcHeightfield* m_solid = nullptr;
    rcCompactHeightfield* m_chf = nullptr;
    rcPolyMesh* m_pmesh = nullptr;
    rcPolyMeshDetail* m_dmesh = nullptr;
    rcContourSet* m_cset = nullptr;
  
  };

}
