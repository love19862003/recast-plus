/************************************************
 * \file NavTool.cpp
 * \date 2019/01/16 16:50
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
#include "NavTool.h"
#include "DetourCrowd.h"
#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "NavResource.h"
#include "DebugDraw.h"
#include "RecastDebugDraw.h"

namespace NavSpace{

  inline unsigned int nextPow2(unsigned int v){
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
  }

  inline unsigned int ilog2(unsigned int v){
    unsigned int r;
    unsigned int shift;
    r = (v > 0xffff) << 4; v >>= r;
    shift = (v > 0xff) << 3; v >>= shift; r |= shift;
    shift = (v > 0xf) << 2; v >>= shift; r |= shift;
    shift = (v > 0x3) << 1; v >>= shift; r |= shift;
    r |= (v >> 1);
    return r;
  }


  static bool isectSegAABB(const float* sp, const float* sq, const float* amin, const float* amax, float& tmin, float& tmax){
    static const float EPS = 1e-6f;

    float d[3];
    d[0] = sq[0] - sp[0];
    d[1] = sq[1] - sp[1];
    d[2] = sq[2] - sp[2];
    tmin = 0.0;
    tmax = 1.0f;

    for (int i = 0; i < 3; i++){
      if (fabsf(d[i]) < EPS){
        if (sp[i] < amin[i] || sp[i] > amax[i])
          return false;
      } else{
        const float ood = 1.0f / d[i];
        float t1 = (amin[i] - sp[i]) * ood;
        float t2 = (amax[i] - sp[i]) * ood;
        if (t1 > t2){ float tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return false;
      }
    }

    return true;
  }

  NavTool::NavTool(rcContext* ctx):NavScene(), NavManager(), m_ctx(ctx){
    m_crowd = dtAllocCrowd();
  }
  NavTool::~NavTool(){
    dtFreeCrowd(m_crowd);
    m_crowd = nullptr;
  }


  bool NavTool::buildTile(const TileIndex& tile){
    cleanup();
    if (!m_solid)m_solid = rcAllocHeightfield();
    if (!m_chf)m_chf = rcAllocCompactHeightfield();
    if (!m_cset)m_cset = rcAllocContourSet();
    if (!m_pmesh)m_pmesh = rcAllocPolyMesh();
    if (!m_dmesh)m_dmesh = rcAllocPolyMeshDetail();

    TileNavCache cache = buildTileCache(m_ctx, &m_cfg, m_solid, m_chf, m_pmesh, m_dmesh, m_cset, tile);
    if (cache.data == nullptr || cache.size == 0){
      return false;
    }

    m_navMesh->removeTile(m_navMesh->getTileRefAt(tile.first, tile.second, 0), 0, 0);
    dtStatus status = m_navMesh->addTile(cache.data, cache.size, DT_TILE_FREE_DATA, 0, 0);
    if (dtStatusFailed(status)){
      dtFree(cache.data);
      return false;
    }
    return true;
  }

  bool NavTool::reset(){
    m_objects.clear();
    m_meshs.clear();
    m_sceneFile = "";
    
    if (m_navMesh) dtFreeNavMesh(m_navMesh);
    if (m_navQuery) dtFreeNavMeshQuery(m_navQuery);
    m_navMesh = nullptr;
    m_navQuery = nullptr;

    offMeshReset();
    return true;
  }
  bool NavTool::load(const std::string& file){
    m_sceneFile =  file;
    if (m_objects.hasData(INVALID_MOBJ_ID)){
      return false;
    }
    //.obj || .mesh  + .voc
    if (hasMagicTag(file, OBJ_TAG) || hasMagicTag(file, MESH_TAG)){
      std::string volFile = VOC_PATH +  setMagicTag(file, VOLUMECONN_TAG);
      FILE* fp = fopen(volFile.data(), "rb");
      if (!fp) {volFile.clear();}
      else{ fclose(fp); } 

      std::string inFile = file;
      if (hasMagicTag(file, OBJ_TAG)){ inFile = OBJECT_PATH + inFile; }
      if (hasMagicTag(file, MESH_TAG)){ inFile = MESH_PATH + inFile; }

      auto ptr = NavResource::genObject(inFile, volFile);
      if (!ptr){
        assert(false);
        return false;
      }
    
      if( m_objects.addData(ptr->id(), ptr)){
        dtVcopy(m_setting.navBmin, ptr->m_bouns.bmin.data());
        dtVcopy(m_setting.navBmax, ptr->m_bouns.bmax.data());
        return true;
      }
      return false;
    }

    //.map
    if (hasMagicTag(file, MAP_TAG)){
      return setScene(file);
    }

    //.dump
    if (hasMagicTag(file, DUMP_TAG)){
      m_dumpFile = file;
      bool res = loadDump(DUMP_PATH + file);
      assert(res);
      return res;
    }

    return false;
  }

  NavScene::TileIndex  NavTool::getTileIndex(const float* pos) const{
    const float* bmin = m_setting.navBmin;
    const float* bmax = m_setting.navBmax;

    const float ts = m_setting.tileSize*m_setting.cellSize;
    const int tx = (int)((pos[0] - bmin[0]) / ts);
    const int ty = (int)((pos[2] - bmin[2]) / ts);
    return std::make_pair(tx, ty);
  }

  int NavTool::tileBit(int v) const{
    int tileBits =  std::min<int>((int)ilog2(nextPow2(v)), 14);
    if (tileBits > 14) tileBits = 14;
    return tileBits;
  }
  bool NavTool::build(){

    if (m_objects.size() <= 0){
      return false;
    }
    dtFreeNavMesh(m_navMesh);
    m_navMesh = dtAllocNavMesh();

    const float* bmin = m_setting.navBmin;
    const float* bmax = m_setting.navBmax;
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_setting.cellSize, &gw, &gh);
    const int ts = (int)m_setting.tileSize;
    const int tw = (gw + ts - 1) / ts;
    const int th = (gh + ts - 1) / ts;
    const float tcs = m_setting.tileSize*m_setting.cellSize;

    dtNavMeshParams params;
    rcVcopy(params.orig, bmin);
    params.tileWidth = tcs;
    params.tileHeight = tcs;
    int tileBits = tileBit(tw*th);
    int polyBits = 22 - tileBits;
    params.maxTiles =  1 << tileBits;
    params.maxPolys = 1 << polyBits;


    if (tw * th > params.maxTiles){
      return false;
    }

    dtStatus status = m_navMesh->init(&params);
    if (dtStatusFailed(status)){
      return false;
    }

    if (nullptr == m_navQuery){
      m_navQuery = dtAllocNavMeshQuery();
    }
    status = m_navQuery->init(m_navMesh, 2048);
    if (dtStatusFailed(status)){
      return false;
    }

    for (int y = 0; y < th; ++y){
      for (int x = 0; x < tw; ++x){
        buildTile(std::make_pair(x, y));
      }
    }
    return true;
  }

  void NavTool::cleanup(){
    rcFreeHeightField(m_solid);
    m_solid = nullptr;
    rcFreeCompactHeightfield(m_chf);
    m_chf = nullptr;
    rcFreeContourSet(m_cset);
    m_cset = nullptr;
    rcFreePolyMesh(m_pmesh);
    m_pmesh = nullptr;
    rcFreePolyMeshDetail(m_dmesh);
    m_dmesh = nullptr;
  }

  void NavTool::saveMapNavMesh(){
    if (!m_navMesh){ return;}
   
    bool res = saveNavMesh(NAV_PATH + m_sceneFile);
    assert(res);
  }

  void NavTool::loadMapNavMesh(){
    auto file = m_dumpFile;
    if (file.empty()){
      file = m_sceneFile;
    }
    file = setMagicTag(file, NAVMESH_TAG);
    loadNavMesh(file);
  }

  void NavTool::saveDumpScene(){
    auto dumpFile = m_dumpFile;
    if (dumpFile.empty()){
      dumpFile = m_sceneFile;
    }
    auto file = setMagicTag(dumpFile, DUMP_TAG);
    assert(!file.empty());
    saveDump(file);
    saveNavMesh(NAV_PATH + file);
  }

  void NavTool::saveMapBin(){
    assert(m_objects.hasData(INVALID_MOBJ_ID));
    auto p = m_objects.getData(INVALID_MOBJ_ID);
    if (!p){
      return;
    }
    bool res = p->saveMap(MAP_PATH + m_sceneFile);
    assert(res);
  }

  void NavTool::saveMeshBin(const MeshPtr& ptr){
    assert(ptr);
    if (!ptr){
      return;
    }
   bool res = ptr->saveMesh(ptr->name());
   assert(res);
  }

  void  NavTool::addMesh(const std::string& file){
    ++m_nextMeshId;
    std::string inFile = file;
    if (hasMagicTag(file, OBJ_TAG)){ inFile = OBJECT_PATH + inFile; }
    if (hasMagicTag(file, MESH_TAG)){ inFile = MESH_PATH + inFile; }
    MeshPtr p = Mesh::loadMesh(m_nextMeshId, inFile);
    if (p){
      m_meshs.addData(p->id(), p);
    }else{
      assert(false);
    }
  }

  void NavTool::removeObject(const float* s, const float* e){
    MObjId id = hitTestMesh(s, e);
    if (id != INVALID_MOBJ_ID){
      m_objects.eraseData(id);
    }

  }

  void NavTool::addObject(const float* pos, float scale, float o, MeshId id){
    auto ptr = m_meshs.getData(id);
    if (!ptr){
      return;
    }
    ++m_nextObjId;
    WorldItem wpos;
    wpos.m_mesh = id;
    wpos.m_o = o;
    wpos.m_scale = scale;
    rcVcopy(wpos.m_pos.data(), pos);
    ObjectPtr obj = std::make_shared<MeshObject>(m_nextObjId, ptr, wpos);
    if (!obj){
      return;
    }
    m_objects.addData(obj->id(), obj);
    
    // only for test 
    //rcVmax(m_setting.navBmax, obj->m_bouns.bmax.data());
    //rcVmin(m_setting.navBmin, obj->m_bouns.bmin.data());
  
    float delMin[3], delMax[3];
    rcVsub(delMin, m_setting.navBmin, obj->m_bouns.bmin.data());
    rcVsub(delMax, m_setting.navBmax, obj->m_bouns.bmax.data());

    assert(delMin[0] <= 0.f && delMin[1] <= 0.f && delMin[3] <= 0.f);
    assert(delMax[0] >= 0.f && delMax[1] >= 0.f && delMax[3] >= 0.f);

    if (!m_navMesh){
      return;
    }

    m_ctx->resetLog();
    m_ctx->resetTimers();
    //rebuild tile 
    const float tcs = m_setting.cellSize * m_setting.tileSize;
    //float delMin[3], delMax[3];
    dtVsub(delMin, obj->m_bouns.bmin.data(), m_setting.navBmin);
    dtVsub(delMax, obj->m_bouns.bmax.data(), m_setting.navBmin);
    int xMin = std::floor(delMin[0] / tcs);
    int yMin = std::floor(delMin[2] / tcs);
    int xMax = std::ceil(delMax[0] / tcs);
    int yMax = std::ceil(delMax[2] / tcs);

    for (int x = xMin; x < xMax; x++){
      for(int y = yMin; y < yMax; y++){
        buildTile(std::make_pair(x, y));
      }
    }

  }

  void NavTool::removeTile(const TileIndex& index){
    assert(m_navMesh);
    if (!m_navMesh) return;
    m_navMesh->removeTile(m_navMesh->getTileRefAt(index.first, index.second, 0), nullptr, 0);
  }

  void NavTool::removeAll(){
    const float* bmin = m_setting.navBmin;
    const float* bmax = m_setting.navBmax;
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_setting.cellSize, &gw, &gh);
    const int ts = (int)m_setting.tileSize;
    const int tw = (gw + ts - 1) / ts;
    const int th = (gh + ts - 1) / ts;
    const float tcs = m_setting.tileSize*m_setting.cellSize;
    for (int y = 0; y < th; ++y){
      for (int x = 0; x < tw; ++x){
        removeTile(std::make_pair(x, y));
      }
    }
  }

  void NavTool::deleteConvexVolume(int i){
    auto ptr = getScene();
    if (!ptr){
      return;
    }

    if (ptr->m_volumeOffConn.m_volumes.remove(i)){
     /* offMeshConBuild();*/
    }

  }

  void NavTool::addConvexVolume(const float* verts, const int nverts, const float minh, const float maxh, unsigned char area){
    auto ptr = getScene();
    if (!ptr){
      return;
    }
    Volume vol;
    memset(&vol, 0, sizeof(Volume));
    memcpy(vol.verts, verts, sizeof(float) * 3 * nverts);
    vol.hmin = minh;
    vol.hmax = maxh;
    vol.nverts = nverts;
    vol.area = area;
    if (!ptr->m_volumeOffConn.m_volumes.add(&vol)){
      assert(false);
    }
  }

  void NavTool::addOffMeshConnection(const float* spos, const float* epos, const float rad,
                            unsigned char bidir, unsigned char area, unsigned short flags){

    auto ptr = getScene();
    if (!ptr){
      return;
    }

    OffMeshCon off;
    off.rad = rad;
    off.dir = bidir;
    off.area = area;
    off.flag = flags;
    off.id= 100000 + ptr->m_volumeOffConn.m_offCons.count();
    rcVcopy(&off.verts[0], spos);
    rcVcopy(&off.verts[3], epos);

    if (ptr->m_volumeOffConn.m_offCons.add(&off)){
       offMeshConBuild();
    }else{
      assert(false);
    }
  }
  void NavTool::deleteOffMeshConnection(int i){
    auto ptr = getScene();
    if (!ptr){
      return;
    }
    if (ptr->m_volumeOffConn.m_offCons.remove(i)){
      offMeshConBuild();
    } else{
      assert(false);
    }
  }

  static void drawOffConn(duDebugDraw* dd, bool hilight, const OffMeshCon* con, size_t index){
    unsigned int conColor = duRGBA(192, 0, 128, 192);
    unsigned int baseColor = duRGBA(0, 0, 0, 64);
    const float* v = con->verts;
    dd->begin(DU_DRAW_LINES, 2.0f);
    dd->vertex(v[0], v[1], v[2], baseColor);
    dd->vertex(v[0], v[1] + 0.2f, v[2], baseColor);

    dd->vertex(v[3], v[4], v[5], baseColor);
    dd->vertex(v[3], v[4] + 0.2f, v[5], baseColor);

    duAppendCircle(dd, v[0], v[1] + 0.1f, v[2], con->rad, baseColor);
    duAppendCircle(dd, v[3], v[4] + 0.1f, v[5], con->rad, baseColor);

    if (hilight){
      duAppendArc(dd, v[0], v[1], v[2], v[3], v[4], v[5], 0.25f,
        (con->dir & 1) ? 0.6f : 0.0f, 0.6f, conColor);
    }
    dd->end();
  }
  void NavTool::drawOffConPools(duDebugDraw* dd, bool hilight){
    
    dd->depthMask(false);
   
    m_objects.forEachValue([&](const ObjectPtr& ptr){
      ptr->m_volumeOffConn.m_offCons.call(std::bind(&drawOffConn, dd, hilight, std::placeholders::_1, std::placeholders::_2));
    });
   
    dd->depthMask(true);
  }

  static void drawVolumePool(struct duDebugDraw* dd, const VolumePool& pool){
    dd->depthMask(false);

    dd->begin(DU_DRAW_TRIS);

    pool.call([dd](const Volume* vol, size_t index){
      unsigned int col = duTransCol(dd->areaToCol(vol->area), 32);
      for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++){
        const float* va = &vol->verts[k * 3];
        const float* vb = &vol->verts[j * 3];

        dd->vertex(vol->verts[0], vol->hmax, vol->verts[2], col);
        dd->vertex(vb[0], vol->hmax, vb[2], col);
        dd->vertex(va[0], vol->hmax, va[2], col);

        dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
        dd->vertex(va[0], vol->hmax, va[2], col);
        dd->vertex(vb[0], vol->hmax, vb[2], col);

        dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
        dd->vertex(vb[0], vol->hmax, vb[2], col);
        dd->vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
      }
    });

    dd->end();

    dd->begin(DU_DRAW_LINES, 2.0f);
    pool.call([dd](const Volume* vol, size_t index){
      unsigned int col = duTransCol(dd->areaToCol(vol->area), 220);
      for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++){
        const float* va = &vol->verts[k * 3];
        const float* vb = &vol->verts[j * 3];
        dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
        dd->vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
        dd->vertex(va[0], vol->hmax, va[2], col);
        dd->vertex(vb[0], vol->hmax, vb[2], col);
        dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
        dd->vertex(va[0], vol->hmax, va[2], col);
      }
    });
    dd->end();

    dd->begin(DU_DRAW_POINTS, 3.0f);
    pool.call([dd](const Volume* vol, size_t index){
      unsigned int col = duDarkenCol(duTransCol(dd->areaToCol(vol->area), 220));
      for (int j = 0; j < vol->nverts; ++j){
        dd->vertex(vol->verts[j * 3 + 0], vol->verts[j * 3 + 1] + 0.1f, vol->verts[j * 3 + 2], col);
        dd->vertex(vol->verts[j * 3 + 0], vol->hmin, vol->verts[j * 3 + 2], col);
        dd->vertex(vol->verts[j * 3 + 0], vol->hmax, vol->verts[j * 3 + 2], col);
      }
    });
    dd->end();


    dd->depthMask(true);
  }

  void NavTool::drawVolumes(struct duDebugDraw* dd, bool hilight){
    m_objects.forEachValue([dd](const ObjectPtr& ptr){
      drawVolumePool(dd, ptr->m_volumeOffConn.m_volumes);
    });
  }





  static void drawTreeBox(TreeNode* node, struct duDebugDraw* dd, float hmin, float hmax, int& icol){
  
    if (!node){
      return;
    }

    if (node->leaf){
      icol++;
      //int c = col * 100 % 255;
      const unsigned int col = duIntToCol(icol * 10, 64);
      unsigned int fcol[6];
      duCalcBoxColors(fcol, col, col);
      duDebugDrawBox(dd, node->bouns.bmin[0] + 0.1f, hmin, node->bouns.bmin[1] + 0.1, node->bouns.bmax[0] - 0.1, hmax, node->bouns.bmax[1] - 0.1, fcol /*duRGBA(255, c, c, 128), 1.0f*/);
    }

    for (auto& c : node->child){
      drawTreeBox(c, dd, hmin, hmax, icol);
    }
   
    
  }

  void NavTool::drawMeshObjects(struct duDebugDraw* dd){
    const float texScale = 1.0f / (m_setting.cellSize * 10.0f);
    m_objects.forEachValue([dd,texScale, this](const ObjectPtr& ptr){
      duDebugDrawTriMeshSlope(dd, ptr->m_verts.pool(), ptr->m_verts.count(),
                              ptr->m_tris.pool(),  ptr->m_normals.pool(), ptr->m_tris.count(),
                              m_setting.agentMaxSlope, texScale);


      if (m_drawBoxTree){
        int col = 0;
        drawTreeBox(ptr->m_tree, dd, ptr->m_bouns.bmin[1], ptr->m_bouns.bmax[1], col);
      }
      
      
    });
  }

  MObjId NavTool::hitTestMesh(const float* s, const float* e) {
    MObjId res = INVALID_MOBJ_ID;
    float tmin = FLT_MAX;
    m_objects.forEachValue([&res, &tmin, s, e, this](const ObjectPtr& ptr){
      if (ptr->id() == INVALID_MOBJ_ID){
        return;
      }
      float t0, t1;
      if (isectSegAABB(s, e, ptr->m_bouns.bmin.data(), ptr->m_bouns.bmax.data(), t0, t1)){
        if (t0 < tmin){
          tmin = t0;
          res = ptr->id();
        }
      }
    });
    return res;
  }

  bool NavTool::raycastMesh(float* src, float* dst, float& tmin){
    bool res = false;
    m_objects.forEachValue([&res, &tmin, src, dst](const ObjectPtr& ptr){
      if (ptr){
        float t = 1.f;
        bool r = ptr->raycastMesh(src, dst, t);
        if (r){ 
          res = true;
          if (t < tmin){ tmin = t; }
        }
      }
    });
    return res;
  }


  bool NavTool::setScenePtr(ObjectPtr ptr){
    if (m_objects.hasData(INVALID_MOBJ_ID)){
      assert(false);
      return false;
    }
    if (!ptr){
      assert(false);
      return false;
    }
    bool res = m_objects.addData(ptr->id(), ptr);
    dtVcopy(m_setting.navBmin, ptr->m_bouns.bmin.data());
    dtVcopy(m_setting.navBmax, ptr->m_bouns.bmax.data());
    assert(res);
    return res;
  }

}