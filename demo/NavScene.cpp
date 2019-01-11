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

#include "NacScene.h"
#include "DetourCommon.h"
#include "GuardFunction.h"
#include "Recast.h"
#include "RecastAlloc.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
namespace NavSpace{


  NavScene::TileNacCache NavScene::buildTile(rcContext* ctx, const TileIndex& tile){
    TileNacCache cache;
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

    buildmax[0] = bmin[0] + tile.first * tcs + tcs;
    buildmax[1] = bmax[1];
    buildmax[2] = bmin[2] + tile.second * tcs + tcs;

    unsigned char* data = buildTileReal(ctx, tile, buildmin, buildmax, cache.size);
    if(data){
      cache.data = data;
    }


    return std::move(cache);
  }

  unsigned char*  NavScene::buildTileReal(rcContext* ctx, const TileIndex& tile, const float bmin[3], const float bmax[3], size_t& dataSize){
    if (m_objects.size() <= 0){
      return nullptr;
    }
    auto scene = m_objects.getData(INVALID_MOBJ_ID);
    if (!scene){
      return nullptr;
    }

    rcHeightfield* solid = rcAllocHeightfield();
    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    rcPolyMesh* pmesh = rcAllocPolyMesh();
    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    rcContourSet* cset = rcAllocContourSet();

    Utility::GuardFun exitFun([&](){
      rcFreeHeightField(solid);
      rcFreeCompactHeightfield(chf);
      rcFreePolyMesh(pmesh);
      rcFreePolyMeshDetail(dmesh);
      rcFreeContourSet(cset);
    });

    rcConfig cfg;
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
    m_objects.forEachValue([&](const ObjectPtr& obj){
      if(!obj->rasterizeTriangles(ctx, cfg, *solid)){
        ctx->log(RC_LOG_ERROR, "Building navigation:rasterizeTriangles error");
        error = true;
      }
    });

    if (error){
      return nullptr;
    }

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
    scene->markVolume(ctx, *chf);


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
      return 0;
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
      params.offMeshConVerts = scene->getOffMeshConnectionVerts();
      params.offMeshConRad = scene->getOffMeshConnectionRads();
      params.offMeshConDir = scene->getOffMeshConnectionDirs();
      params.offMeshConAreas = scene->getOffMeshConnectionAreas();
      params.offMeshConFlags = scene->getOffMeshConnectionFlags();
      params.offMeshConUserID = scene->getOffMeshConnectionId();
      params.offMeshConCount = scene->getOffMeshConnectionCount();
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
        return 0;
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

}