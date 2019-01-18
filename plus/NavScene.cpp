/************************************************
 * \file NavScene.cpp
 * \date 2019/01/11 16:49
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

#include "NavScene.h"
#include "DetourCommon.h"
#include "GuardFunction.h"
#include "Recast.h"
#include "RecastAlloc.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "MyJson.h"
#include "NavResource.h"
namespace NavSpace{

  typedef Pool<float, 3 * 2> OffMeshConVertsPool;
  typedef Pool<float> OffMeshConRadPool;
  typedef Pool<unsigned char> OffMeshConDirPool;
  typedef Pool<unsigned char> OffMeshConAreaPool;
  typedef Pool<unsigned short> OffMeshConFlagPool;
  typedef Pool<unsigned int> OffMeshConIdPool;
  struct BuildOffMeshData{
    OffMeshConVertsPool verts;
    OffMeshConRadPool rads;
    OffMeshConDirPool dirs;
    OffMeshConFlagPool flags;
    OffMeshConIdPool ids;
    OffMeshConAreaPool areas;
    int count;
  };


  NavScene::NavScene(){
  
  }
  NavScene::~NavScene(){
  }

  bool NavScene::setScene(const std::string& file){
    if (!hasMagicTag(file, MAP_TAG) || m_objects.hasData(INVALID_MOBJ_ID)){
      assert(false);
      return false;
    }
    auto ptr = NavResource::readObject(MAP_PATH + file);
    if (!ptr){
      assert(false);
      return false;
    }
    bool res = m_objects.addData(ptr->id(), ptr);
    dtVcopy(m_setting.navBmin, ptr->m_bouns.bmin.data());
    dtVcopy(m_setting.navBmax, ptr->m_bouns.bmax.data());
    m_sceneFile = file;
    return res;
  }
  bool NavScene::saveDump(const std::string& name){
    auto file = DUMP_PATH + setMagicTag(name, DUMP_TAG);
    try{
        Utility::MyJson json;
        json.set("setting.cellSize", m_setting.cellSize);
        json.set("setting.cellHeight", m_setting.cellHeight);
        json.set("setting.agentHeight", m_setting.agentHeight);
        json.set("setting.agentRadius", m_setting.agentRadius);
        json.set("setting.agentMaxClimb", m_setting.agentMaxClimb);
        json.set("setting.agentMaxSlope", m_setting.agentMaxSlope);
        json.set("setting.regionMinSize", m_setting.regionMinSize);
        json.set("setting.regionMergeSize", m_setting.regionMergeSize);
        json.set("setting.edgeMaxLen", m_setting.edgeMaxLen);
        json.set("setting.edgeMaxError", m_setting.edgeMaxError);
        json.set("setting.vertsPerPoly", m_setting.vertsPerPoly);
        json.set("setting.detailSampleDist", m_setting.detailSampleDist);
        json.set("setting.detailSampleMaxError", m_setting.detailSampleMaxError);
        json.set("setting.partitionType", m_setting.partitionType);
        json.set("setting.filterLowHangingObstacles", m_setting.filterLowHangingObstacles);
        json.set("setting.filterLedgeSpans", m_setting.filterLedgeSpans);
        json.set("setting.filterWalkableLowHeightSpans", m_setting.filterWalkableLowHeightSpans);
        json.set("setting.tileSize", m_setting.tileSize);
        json.set("setting.navBmin.x", m_setting.navBmin[0]);
        json.set("setting.navBmin.y", m_setting.navBmin[1]);
        json.set("setting.navBmin.z", m_setting.navBmin[2]);
        json.set("setting.navBmax.x", m_setting.navBmax[0]);
        json.set("setting.navBmax.y", m_setting.navBmax[1]);
        json.set("setting.navBmax.z", m_setting.navBmax[2]);

        std::vector<Utility::MyJson> meshs;
        m_meshs.forEachValue([&meshs](const MeshPtr& ptr){
          if (!ptr) return;
          Utility::MyJson tree;
          tree.set("mesh", ptr->name());
          tree.set("id", ptr->id());
          meshs.push_back(tree);
        });

        json.set("scene", sceneFile());

        json.addTreeArray("mesh", meshs);
        std::vector<Utility::MyJson> objects;
        m_objects.forEachValue([&objects](const ObjectPtr& ptr){
          if (!ptr || ptr->id() == INVALID_MOBJ_ID) return;
          const WorldItem& item = ptr->item();
          Utility::MyJson tree;
          tree.set("id", ptr->id());
          tree.set("world.mesh", item.m_mesh);
          tree.set("world.x", item.m_pos[0]);
          tree.set("world.y", item.m_pos[1]);
          tree.set("world.z", item.m_pos[2]);
          tree.set("world.o", item.m_o);
          tree.set("world.scale", item.m_scale);
          objects.push_back(tree);
        });
        json.addTreeArray("object", objects);
        bool res = json.saveToFile(file);
        assert(res);
        return res;
    }catch(...){
      return false;
    }
  }
  bool NavScene::loadDump(const std::string& file){
    if (!hasMagicTag(file, DUMP_TAG)){
      return false;
    }
    try{
      Utility::MyJson json(DUMP_PATH + file);
      m_setting.cellSize = json.get("setting.cellSize", m_setting.cellSize);
      m_setting.cellHeight = json.get("setting.cellHeight", m_setting.cellHeight);
      m_setting.agentHeight = json.get("setting.agentHeight", m_setting.agentHeight);
      m_setting.agentRadius = json.get("setting.agentRadius", m_setting.agentRadius);
      m_setting.agentMaxClimb = json.get("setting.agentMaxClimb", m_setting.agentMaxClimb);
      m_setting.agentMaxSlope = json.get("setting.agentMaxSlope", m_setting.agentMaxSlope);
      m_setting.regionMinSize = json.get("setting.regionMinSize", m_setting.regionMinSize);
      m_setting.regionMergeSize = json.get("setting.regionMergeSize", m_setting.regionMergeSize);
      m_setting.edgeMaxLen = json.get("setting.edgeMaxLen", m_setting.edgeMaxLen);
      m_setting.edgeMaxError = json.get("setting.edgeMaxError", m_setting.edgeMaxError);
      m_setting.vertsPerPoly = json.get("setting.vertsPerPoly", m_setting.vertsPerPoly);
      m_setting.detailSampleDist = json.get("setting.detailSampleDist", m_setting.detailSampleDist);
      m_setting.detailSampleMaxError = json.get("setting.detailSampleMaxError", m_setting.detailSampleMaxError);
      m_setting.partitionType = json.get("setting.partitionType", m_setting.partitionType);
      m_setting.filterLowHangingObstacles = json.get("setting.filterLowHangingObstacles", m_setting.filterLowHangingObstacles);
      m_setting.filterLedgeSpans = json.get("setting.filterLedgeSpans", m_setting.filterLedgeSpans);
      m_setting.filterWalkableLowHeightSpans = json.get("setting.filterWalkableLowHeightSpans", m_setting.filterWalkableLowHeightSpans);
      m_setting.tileSize = json.get("setting.tileSize", m_setting.tileSize);
      m_setting.navBmin[0] = json.get("setting.navBmin.x", m_setting.navBmin[0]);
      m_setting.navBmin[1] = json.get("setting.navBmin.y", m_setting.navBmin[1]);
      m_setting.navBmin[2] = json.get("setting.navBmin.z", m_setting.navBmin[2]);
      m_setting.navBmax[0] = json.get("setting.navBmax.x", m_setting.navBmax[0]);
      m_setting.navBmax[1] = json.get("setting.navBmax.y", m_setting.navBmax[1]);
      m_setting.navBmax[2] = json.get("setting.navBmax.z", m_setting.navBmax[2]);

      typedef std::tuple<MeshId, std::string> MeshData;
      std::vector<MeshData> meshs;
      json.getGroup<MeshData>("mesh", meshs, [](const Utility::MyJson& tree){
        MeshData data;
        std::get<0>(data) = tree.get("id", MeshId(INVALID_MESH_ID));
        std::get<1>(data) = tree.get("mesh", std::string());
        return data;
      });

      std::string scene = json.get("scene", std::string());
      if (!setScene(scene)){ assert(false); return false; }

      typedef std::tuple<MObjId, WorldItem> ObjectData;
      std::vector<ObjectData> objects;
      json.getGroup<ObjectData>("object", objects, [](const Utility::MyJson& tree){
        ObjectData data;
        std::get<0>(data) = tree.get("id", MObjId(INVALID_MOBJ_ID));
        WorldItem& item = std::get<1>(data);
        item.m_mesh = tree.get("world.mesh", MeshId(INVALID_MESH_ID));
        item.m_pos[0] = tree.get("world.x", 0.f);
        item.m_pos[1] = tree.get("world.y", 0.f);
        item.m_pos[2] = tree.get("world.z", 0.f);
        item.m_o = tree.get("world.o", 0.f);
        item.m_scale = tree.get("world.scale", 1.f);
        assert(std::get<0>(data) != INVALID_MOBJ_ID);
        return data;
      });

      std::sort(meshs.begin(), meshs.end(), [](const MeshData&l, const MeshData& r){ return std::get<0>(l) < std::get<0>(r); });
      std::sort(objects.begin(), objects.end(), [](const ObjectData&l, const ObjectData& r){ return std::get<0>(l) < std::get<0>(r); });

      for (auto& mesh : meshs){
        MeshPtr m = Mesh::loadMesh(std::get<0>(mesh), std::get<1>(mesh));
        assert(m);
        if (!m){ assert(false); continue; }
        if (!m_meshs.addData(m->id(), m)){ assert(false); continue; }
      }

      for (auto& obj : objects){
        MObjId id = std::get<0>(obj);
        const WorldItem& item = std::get<1>(obj);
        assert(id != INVALID_MOBJ_ID);
        if (id == INVALID_MOBJ_ID){ assert(false); continue; }
        MeshPtr ptr = m_meshs.getData(item.m_mesh);
        if (!ptr){ assert(false); continue; }
        ObjectPtr object = std::make_shared<MeshObject>(id, ptr, item);
        assert(object);
        if (!object){ assert(false); continue; }
        if (!m_objects.addData(id, object)){ assert(false); continue; }
      }

      return true;
    } catch (...){ 
      return false;
    }
  }

  NavScene::TileNavCache NavScene::buildTileCache(rcContext* ctx,
                                                  rcConfig* cfg,
                                                  rcHeightfield* solid,
                                                  rcCompactHeightfield* chf,
                                                  rcPolyMesh* pmesh,
                                                  rcPolyMeshDetail* dmesh,
                                                  rcContourSet* cset,
                                                  const TileIndex& tile){
    TileNavCache cache;
    cache.index = tile;
    cache.data = nullptr;
    cache.size = 0;

    const float* bmin = m_setting.navBmin;
    const float* bmax = m_setting.navBmax;
    const float tcs = m_setting.tileSize * m_setting.cellSize;

    float buildmin[3]; 
    float buildmax[3];
   
    buildmin[0] = bmin[0] + tile.first * tcs;
    buildmin[1] = bmin[1];
    buildmin[2] = bmin[2] + tile.second * tcs;

    buildmax[0] = bmin[0] + (tile.first  + 1)* tcs;
    buildmax[1] = bmax[1];
    buildmax[2] = bmin[2] + (tile.second  + 1)* tcs ;

    unsigned char* data = buildTileReal(ctx, cfg, solid, chf, pmesh, dmesh, cset, tile, buildmin, buildmax, cache.size);
    if(data){
      cache.data = data;
    }
    return std::move(cache);
  }

  unsigned char*  NavScene::buildTileReal(rcContext* ctx,
                                          rcConfig* cf,
                                          rcHeightfield* solid,
                                          rcCompactHeightfield* chf,
                                          rcPolyMesh* pmesh,
                                          rcPolyMeshDetail* dmesh,
                                          rcContourSet* cset,
                                          const TileIndex& tile, 
                                          const float bmin[3],
                                          const float bmax[3],
                                          size_t& dataSize){
    if (m_objects.size() <= 0){
      return nullptr;
    }

    rcConfig& cfg = *cf;
    memset(&cfg, 0, sizeof(rcConfig));
    cfg.cs = m_setting.cellSize;
    cfg.ch = m_setting.cellHeight;
    cfg.walkableSlopeAngle = m_setting.agentMaxSlope;
    cfg.walkableHeight = (int)ceilf(m_setting.agentHeight / cfg.ch);
    cfg.walkableClimb = (int)floorf(m_setting.agentMaxClimb / cfg.ch);
    cfg.walkableRadius = (int)ceilf(m_setting.agentRadius / cfg.cs);
    cfg.maxEdgeLen = (int)(m_setting.edgeMaxLen / m_setting.cellSize);
    cfg.maxSimplificationError = m_setting.edgeMaxError;
    cfg.minRegionArea = (int)rcSqr(m_setting.regionMinSize);		// Note: area = size*size
    cfg.mergeRegionArea = (int)rcSqr(m_setting.regionMergeSize);	// Note: area = size*size
    cfg.maxVertsPerPoly = (int)m_setting.vertsPerPoly;
    cfg.tileSize = (int)m_setting.tileSize;
    cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
    cfg.width = cfg.tileSize + cfg.borderSize * 2;
    cfg.height = cfg.tileSize + cfg.borderSize * 2;
    cfg.detailSampleDist = m_setting.detailSampleDist < 0.9f ? 0 : m_setting.cellSize * m_setting.detailSampleDist;
    cfg.detailSampleMaxError = m_setting.cellHeight * m_setting.detailSampleMaxError;
    rcVcopy(cfg.bmin, bmin);
    rcVcopy(cfg.bmax, bmax);
    cfg.bmin[0] -= cfg.borderSize*cfg.cs;
    cfg.bmin[2] -= cfg.borderSize*cfg.cs;
    cfg.bmax[0] += cfg.borderSize*cfg.cs;
    cfg.bmax[2] += cfg.borderSize*cfg.cs;

    ctx->resetTimers();
    ctx->startTimer(RC_TIMER_TOTAL);
    ctx->log(RC_LOG_PROGRESS, "Building navigation:");
    ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", cfg.width, cfg.height);

    bool filterLowHangingObstacles = m_setting.filterLowHangingObstacles;
    bool filterWalkableLowHeightSpans = m_setting.filterWalkableLowHeightSpans;
    bool filterLedgeSpans = m_setting.filterLedgeSpans;
    int partitionType = m_setting.partitionType;


    if (!solid || !chf || !pmesh || !dmesh || !cset){
      ctx->log(RC_LOG_ERROR, "Building navigation: memory full");
      return nullptr;
    }
    
    if (!rcCreateHeightfield(ctx, *solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch)){
      ctx->log(RC_LOG_ERROR, "Building navigation:could not create height field");
      return nullptr;
    }

     bool error = false;
     m_objects.forEachValue([&ctx, &cfg, solid, &error](const ObjectPtr& obj){
       if (!obj->rasterizeTriangles(ctx, cfg, *solid)){
         ctx->log(RC_LOG_ERROR, "Building navigation:rasterizeTriangles error");
         error = true;
       }
     });

//     if (error){
//       return nullptr;
//     }

    if (filterLowHangingObstacles)
      rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *solid);
    if (filterLedgeSpans)
      rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *solid);
    if (filterWalkableLowHeightSpans)
      rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *solid);

    if (!rcBuildCompactHeightfield(ctx, cfg.walkableHeight, cfg.walkableClimb, *solid, *chf)){
      ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
      return nullptr;
    }

    if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf)){
      ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
      return nullptr;
    }

    //mark areas
    //or for each ??
    m_objects.forEachValue([ctx, chf](const ObjectPtr& obj){
      obj->markVolume(ctx, *chf);
    });


    if (partitionType == SAMPLE_PARTITION_WATERSHED){
      // Prepare for region partitioning, by calculating distance field along the walkable surface.
      if (!rcBuildDistanceField(ctx, *chf)){
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
        return nullptr;
      }

      // Partition the walkable surface into simple regions without holes.
      if (!rcBuildRegions(ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)){
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
        return nullptr;
      }
    } else if (partitionType == SAMPLE_PARTITION_MONOTONE){
      // Partition the walkable surface into simple regions without holes.
      // Monotone partitioning does not need distancefield.
      if (!rcBuildRegionsMonotone(ctx, *chf, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea)){
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
        return nullptr;
      }
    } else // SAMPLE_PARTITION_LAYERS
    {
      // Partition the walkable surface into simple regions without holes.
      if (!rcBuildLayerRegions(ctx, *chf, cfg.borderSize, cfg.minRegionArea)){
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
        return nullptr;
      }
    }


   
    if (!rcBuildContours(ctx, *chf, cfg.maxSimplificationError, cfg.maxEdgeLen, *cset)){
      ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
      return nullptr;
    }

    if (cset->nconts == 0){
      ctx->log(RC_LOG_ERROR, "buildNavigation: tile: x-%d y-%d cset->nconts = 0.", tile.first, tile.second);
      return nullptr;
    }


    if (!rcBuildPolyMesh(ctx, *cset, cfg.maxVertsPerPoly, *pmesh)){
      ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
      return nullptr;
    }

    if (!rcBuildPolyMeshDetail(ctx, *pmesh, *chf,
                               cfg.detailSampleDist, cfg.detailSampleMaxError,
                               *dmesh)){
      ctx->log(RC_LOG_ERROR, "buildNavigation: Could build polymesh detail.");
      return nullptr;
    }


    unsigned char* navData = 0;
    int navDataSize = 0;
    if (cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON){
      if (pmesh->nverts >= 0xffff){
        // The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
        ctx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", pmesh->nverts, 0xffff);
        return nullptr;
      }

      // Update poly flags from areas.
      for (int i = 0; i < pmesh->npolys; ++i){
        if (pmesh->areas[i] == RC_WALKABLE_AREA)
          pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;

        if (pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
            pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
            pmesh->areas[i] == SAMPLE_POLYAREA_ROAD){
          pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
        } else if (pmesh->areas[i] == SAMPLE_POLYAREA_WATER){
          pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
        } else if (pmesh->areas[i] == SAMPLE_POLYAREA_DOOR){
          pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
        }
      }

      dtNavMeshCreateParams params;
      memset(&params, 0, sizeof(params));
      params.verts = pmesh->verts;
      params.vertCount = pmesh->nverts;
      params.polys = pmesh->polys;
      params.polyAreas = pmesh->areas;
      params.polyFlags = pmesh->flags;
      params.polyCount = pmesh->npolys;
      params.nvp = pmesh->nvp;
      params.detailMeshes = dmesh->meshes;
      params.detailVerts = dmesh->verts;
      params.detailVertsCount = dmesh->nverts;
      params.detailTris = dmesh->tris;
      params.detailTriCount = dmesh->ntris;
      params.offMeshConVerts = getOffMeshConnectionVerts();
      params.offMeshConRad = getOffMeshConnectionRads();
      params.offMeshConDir = getOffMeshConnectionDirs();
      params.offMeshConAreas = getOffMeshConnectionAreas();
      params.offMeshConFlags = getOffMeshConnectionFlags();
      params.offMeshConUserID = getOffMeshConnectionId();
      params.offMeshConCount = getOffMeshConnectionCount();
      params.walkableHeight = m_setting.agentHeight;
      params.walkableRadius = m_setting.agentRadius;
      params.walkableClimb = m_setting.agentMaxClimb;
      params.tileX = tile.first;
      params.tileY = tile.second;
      params.tileLayer = 0;
      rcVcopy(params.bmin, pmesh->bmin);
      rcVcopy(params.bmax, pmesh->bmax);
      params.cs = cfg.cs;
      params.ch = cfg.ch;
      params.buildBvTree = true;

      if (!dtCreateNavMeshData(&params, &navData, &navDataSize)){
        ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
        return nullptr;
      }
    }
    //m_tileMemUsage = navDataSize / 1024.0f;

    ctx->stopTimer(RC_TIMER_TOTAL);

    // Show performance stats.
    //duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
    ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", pmesh->nverts, pmesh->npolys);

   // m_tileBuildTime = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;

    dataSize = navDataSize;
    return navData;
  }

  void NavScene::offMeshReset() const{
    if (m_buildOff){
      m_buildOff->count = 0;
      m_buildOff->verts.reset();
      m_buildOff->rads.reset();
      m_buildOff->ids.reset();
      m_buildOff->dirs.reset();
      m_buildOff->areas.reset();
      m_buildOff->flags.reset();
      m_buildOff->dirs.reset();
    }
  }
  void NavScene::offMeshConBuild() const{
    if (!m_buildOff){
      m_buildOff.reset(new BuildOffMeshData);
    }
    if (!m_buildOff){ return; }
    m_buildOff->count = 0;
    m_buildOff->verts.reset();
    m_buildOff->rads.reset();
    m_buildOff->ids.reset();
    m_buildOff->dirs.reset();
    m_buildOff->areas.reset();
    m_buildOff->flags.reset();
    m_buildOff->dirs.reset();


    auto& cref = m_objects.constRefMap();
    for (auto& pair : cref){
      auto obj = pair.second;
      auto& vol = obj->volumeOffConns();
      m_buildOff->count = m_buildOff->count + vol.m_offCons.count();
      vol.m_offCons.call([this](const OffMeshCon* conn, size_t index){
        m_buildOff->verts.add(conn->verts);
        m_buildOff->rads.add(&conn->rad);
        m_buildOff->ids.add(&conn->id);
        m_buildOff->dirs.add(&conn->dir);
        m_buildOff->areas.add(&conn->area);
        m_buildOff->flags.add(&conn->flag);
        m_buildOff->dirs.add(&conn->dir);
      });
    }
  }

  const float* NavScene::getOffMeshConnectionVerts() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->verts.pool();
  }
  const float* NavScene::getOffMeshConnectionRads() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->rads.pool();
  }
  const unsigned char* NavScene::getOffMeshConnectionDirs() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->dirs.pool();
  }
  const unsigned char* NavScene::getOffMeshConnectionAreas() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->areas.pool();

  }
  const unsigned short* NavScene::getOffMeshConnectionFlags() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->flags.pool();

  }
  const unsigned int* NavScene::getOffMeshConnectionId() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->ids.pool();
  }

  int NavScene::getOffMeshConnectionCount() const{
    return m_buildOff->count;
  }

}