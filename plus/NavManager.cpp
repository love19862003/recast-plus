/************************************************
 * \file NavManager.cpp
 * \date 2019/01/16 10:10
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
#include <fstream>
#include "NavManager.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourNavMeshQuery.h"
#include "NavScene.h"
#include "GuardFunction.h"
#include "NavResource.h"


namespace NavSpace{

  template<typename T, typename ... ARGS >
  T* navAlloc(ARGS ... args ){
    T* t = (T*)dtAlloc(sizeof(T), DT_ALLOC_PERM);
    if (!t) return nullptr;
    return new(t) T(std::forward<ARGS>(args)...);

  }

  template<typename T>
  void navFree(T* t){
    if (!t) return;
    t->~T();
    dtFree(t);
  }

  static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
  static const int NAVMESHSET_VERSION = 1;

  struct NavMeshSetHeader{
    int magic;
    int version;
    int numTiles;
    dtNavMeshParams params;
  };

  struct NavMeshTileHeader{
    dtTileRef tileRef;
    int dataSize;
  };


  NavManager::NavManager() 
    : m_navMesh(nullptr)
    , m_navQuery(nullptr)
    , m_filter(nullptr){
    m_filter = navAlloc<dtQueryFilter>();
    m_filter->setAreaCost(SAMPLE_POLYAREA_GROUND, 1.f);
    m_filter->setAreaCost(SAMPLE_POLYAREA_WATER, 10.f);
    m_filter->setAreaCost(SAMPLE_POLYAREA_ROAD, 1.f);
    m_filter->setAreaCost(SAMPLE_POLYAREA_DOOR, 1.f);
    m_filter->setAreaCost(SAMPLE_POLYAREA_GRASS, 2.f);
    m_filter->setAreaCost(SAMPLE_POLYAREA_JUMP, 15.f);
  } 
  NavManager::~NavManager(){
    if (m_navMesh) dtFreeNavMesh(m_navMesh);
    if (m_navQuery) dtFreeNavMeshQuery(m_navQuery);
    if (m_filter) navFree(m_filter);
    m_navMesh = nullptr;
    m_navQuery = nullptr;
    m_filter = nullptr;
  }

  bool NavManager::loadNavMesh(const std::string& file){
    if (file.empty() || !hasMagicTag(file, NAVMESH_TAG)) return false;
    
    try{
      std::ifstream ifile(NAV_PATH + file, std::ios::in | std::ios::binary);
      {
        Utility::GuardFun exit([&ifile](){ ifile.close(); });
        NavMeshSetHeader header;
        ifile.read((char*)&header, sizeof(NavMeshSetHeader));
        if (header.magic != NAVMESHSET_MAGIC){
          return false;
        }

        if (header.version != NAVMESHSET_VERSION){
          return false;
        }

        m_navMesh = dtAllocNavMesh();
        if (nullptr == m_navMesh){
          return false;
        }
        dtStatus status = m_navMesh->init(&header.params);
        if (dtStatusFailed(status)){
          return false;
        }

        for (int i = 0; i < header.numTiles;++i){
          NavMeshTileHeader tileHeader;
          ifile.read((char*)&tileHeader, sizeof(tileHeader));

          if (0 == tileHeader.tileRef || 0 == tileHeader.dataSize){
            break;
          }

          unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
          if(nullptr == data) break;
          memset(data, 0, tileHeader.dataSize);
          ifile.read((char*)data, tileHeader.dataSize);
          m_navMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
        }

        if (nullptr == m_navQuery){
          m_navQuery = dtAllocNavMeshQuery();
        }
        status = m_navQuery->init(m_navMesh, 2048);
        return dtStatusSucceed(status);
      }
    }catch(...){
      return false;
    }
  }


  bool NavManager::saveNavMesh(const std::string& path) const{
    if (path.empty() || nullptr == m_navMesh) return false;
    auto file = setMagicTag(path, NAVMESH_TAG);
    try{
      std::ofstream ofile(file, std::ios::out | std::ios::binary | std::ios::trunc);
      const dtNavMesh* mesh = m_navMesh;
      NavMeshSetHeader header;
      header.magic = NAVMESHSET_MAGIC;
      header.version = NAVMESHSET_VERSION;
      header.numTiles = 0;
      memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
      for (int i = 0; i < mesh->getMaxTiles(); ++i){
        const dtMeshTile* tile = mesh->getTile(i);
        if(!tile || !tile->header || !tile->dataSize) continue;
        ++header.numTiles;
      }

      ofile.write((const char*)&header, sizeof(NavMeshSetHeader));
      for(int i = 0; i < mesh->getMaxTiles(); ++i){
        const dtMeshTile* tile = mesh->getTile(i);
        if (!tile || !tile->header || !tile->dataSize) continue;
        NavMeshTileHeader tileHeader;
        tileHeader.tileRef = mesh->getTileRef(tile);
        tileHeader.dataSize = tile->dataSize;
        ofile.write((const char*)&tileHeader, sizeof(NavMeshTileHeader));
        ofile.write((const char*)tile->data, tile->dataSize);
      }

      ofile.close();
      return true;
    }catch(...){
      return false;
    }
  }


  bool NavManager::findPath(const NavPos& s, const NavPos& e, std::vector<NavPos>& res) const{
    if (!m_navMesh || !m_navQuery) return false;
    dtPolyRef sRef = getPolyRef(s);
    dtPolyRef eRef = getPolyRef(e);
    if (!sRef || !eRef){ return false; }

    const static int MAX_POLYS = 2048;
    dtPolyRef polys[MAX_POLYS];
    int npolys = 0;
    unsigned char straightPathFlags[MAX_POLYS];
    dtPolyRef straightPolys[MAX_POLYS];
    float straightPathPoints[MAX_POLYS * 3];
    int nstraightCount = 0;
    dtStatus status = m_navQuery->findPath(sRef, eRef, s.data(), e.data(), m_filter, polys, &npolys, MAX_POLYS);
    if (dtStatusFailed(status) || npolys == 0){
      return false;
    }
    float epos[3];
    dtVcopy(epos, e.data());
    if (polys[npolys - 1] != eRef){
      m_navQuery->closestPointOnPoly(polys[npolys - 1], e.data(), epos, nullptr);
    }
    status = m_navQuery->findStraightPath(s.data(), epos, polys, npolys, straightPathPoints, straightPathFlags, straightPolys, &nstraightCount, MAX_POLYS);
    if (dtStatusFailed(status)){
      return false;
    }
    for (int i = 0; i < nstraightCount; ++i){
      NavPos pos;
      dtVcopy(pos.data(), &straightPathPoints[i * 3]);
      res.push_back(pos);
    }
    return res.size() > 0;

  }

  dtPolyRef NavManager::getPolyRef(const NavPos& pos, NavPos* out) const{
    if (!m_navMesh || !m_navQuery){
      return 0;
    }
    float ext[3];
    ext[0] = 2.f;
    ext[1] = 4.f;
    ext[2] = 2.f;
    dtPolyRef res = 0;
    dtStatus s = m_navQuery->findNearestPoly(pos.data(), ext, m_filter, &res, out ? out->data() : nullptr);
    if (dtStatusSucceed(s)){
      return res;
    }
    return 0;
  }

  static constexpr int randomMaxCount(){ return 255; }
  bool NavManager::randomPoint(dtPolyRef& ref, const NavPos& c, float r, float(*frand)(), NavPos& out) const{
    if (ref <= 0){ return false; }
    dtPolyRef randRef = 0;
    dtStatus status = m_navQuery->findRandomPointAroundCircle(ref, c.data(), r, m_filter, frand, &randRef, out.data());
    if (dtStatusFailed(status)){
      return false;
    }
    return true;
  }
  bool NavManager::randomPoint(const NavPos& c, float r, float(*frand)(), NavPos& out)const{
    dtPolyRef ref = getPolyRef(c);
    if (ref <= 0){ return false; }
    int count = randomMaxCount();
    do 
    {
      if (!randomPoint(ref, c, r, frand, out)){return false; }

      if (dtVdist2DSqr(c.data(), out.data()) <=  r*r){return true; }

      --count;
    } while (count > 0);
    out = c;
    return false;
  }

  bool NavManager::randomPoints(const NavPos& c, int count, float r, float(*frand)(), std::vector<NavPos>& out) const{
    dtPolyRef ref = getPolyRef(c);
    if (ref <= 0){ return false; }
    NavPos pos;
    for (int i = 0; i < count; ++i){
      int count = randomMaxCount();
      do{
        if (!randomPoint(ref, c, r, frand, pos)){
          return false;
        }

        if (dtVdist2DSqr(c.data(), pos.data()) <= r*r){
          out.push_back(pos);
          break;
        }
        --count;
      } while (count > 0);
    }
    return out.size() > 0;
  }

  bool NavManager::rayCast(const NavPos&s, const NavPos& e, NavPos& out)const{
    dtPolyRef sRef = getPolyRef(s);
    dtPolyRef eRef = getPolyRef(e);

    if (sRef <= 0) return false;

    const static int MAX_POLYS = 2048;
    dtPolyRef polys[MAX_POLYS];
    int npolys = 0;
    float t = 0.f;
    dtStatus status = m_navQuery->raycast(sRef, s.data(), e.data(), m_filter, &t, out.data(), polys, &npolys, MAX_POLYS);
    assert(dtStatusSucceed(status));
    if (t >= 1.f){
      out = e;
      return true;
    }
    dtVlerp(out.data(), s.data(), e.data(), t);
    if (npolys > 0){
      float h = 0.f;
      status = m_navQuery->getPolyHeight(polys[npolys - 1], out.data(), &h);
      assert(dtStatusSucceed(status));
      out[1] = h;
    }
    return true;
  }

  unsigned short NavManager::getTileFlag(const NavPos& c) const{
    unsigned short res = 0;
    dtPolyRef ref = getPolyRef(c);
    if (ref  > 0 && m_navMesh){
      m_navMesh->getPolyFlags(ref, &res);
    }
    return res;
  }

  bool NavManager::findDistanceToWall(const NavPos&c, float r, float& dis, NavPos& out) const{
    if (!m_navMesh || !m_navQuery) return false;
    dtPolyRef ref = getPolyRef(c);
    if (ref <= 0) return false;
    dis = 0.f;
    float hit[3];
    dtStatus status = m_navQuery->findDistanceToWall(ref, c.data(), r, m_filter, &dis, out.data(), hit);
    if (!dtStatusFailed(status)){
      return false;
    }
    return true;
  }

  bool NavManager::isReachAble(const NavPos& p, float diff, NavPos* out /* = nullptr */) const{
     if (!m_navMesh || !m_navQuery){  return false; }
     NavPos o;
     NavPos* po = out ? out : &o;
     dtPolyRef ref = getPolyRef(p, po);
     if (ref <= 0) return false;
     return dtVdistSqr(po->data(), p.data()) <= diff;
  }

}
