/************************************************
 * \file NacScene.h
 * \date 2019/01/11 16:38
 *
 * \author wufan
 * Contact: love19862003@163.com
 *
 * \brief 
 *
 * TODO: long description
 *
 * \note
*************************************************/
#pragma once
#include "NavObject.h"
#include "MapTemplate.h"
#include "NavCommon.h"

struct rcConfig;
struct rcHeightfield;
struct rcCompactHeightfield;
struct rcPolyMesh;
struct rcPolyMeshDetail;
struct rcContourSet;

namespace NavSpace{
  class NavScene;
  typedef std::shared_ptr<NavScene> ScenePtr;
  struct BuildOffMeshData;
  typedef std::unique_ptr<BuildOffMeshData> BuildOffMeshConPtr;

  class NavScene  : public NonCopyAble
  {
  public:
    enum SCENE_TYPE{
      RUN_TIME,
      RUN_TOOL,
    };
    explicit NavScene(SCENE_TYPE type){;}
    virtual ~NavScene(){}
    
    typedef Utility::ObjPtrMap<MeshId, Mesh> MeshMap;
    typedef Utility::ObjPtrMap<MObjId, MeshObject> ObjectMap;
    typedef std::pair<int, int> TileIndex;
    
    struct TileNavCache {
      TileIndex index;
      unsigned char* data;
      size_t size;
    };
    virtual void handleReander() = 0;
    virtual bool buildTile(const TileIndex& tile) = 0;
  private:
    TileNavCache buildTileCache(rcContext* ctx,
                                rcConfig* cf,
                                rcHeightfield* solid,
                                rcCompactHeightfield* chf,
                                rcPolyMesh* pmesh,
                                rcPolyMeshDetail* dmesh,
                                rcContourSet* cset,
                                const TileIndex& tile);
    unsigned char*  buildTileReal(rcContext* ctx,
                                  rcConfig* cf,
                                  rcHeightfield* solid,
                                  rcCompactHeightfield* chf,
                                  rcPolyMesh* pmesh,
                                  rcPolyMeshDetail* dmesh,
                                  rcContourSet* cset,
                                  const TileIndex& tile,
                                  const float bmin[3],
                                  const float bmax[3],
                                  size_t& dataSize);

    const float* getOffMeshConnectionVerts() const;
    const float* getOffMeshConnectionRads() const;
    const unsigned char* getOffMeshConnectionDirs() const;
    const unsigned char* getOffMeshConnectionAreas() const;
    const unsigned short* getOffMeshConnectionFlags() const;
    const unsigned int* getOffMeshConnectionId() const;
    int getOffMeshConnectionCount() const;

  protected:
    void offMeshConBuild() const;
  protected:
    ObjectMap m_objects;
    MeshMap m_meshs;
    NavSetting m_setting;
    SCENE_TYPE m_type;
    mutable BuildOffMeshConPtr m_buildOff;
  };

}
