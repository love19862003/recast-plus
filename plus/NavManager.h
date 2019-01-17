/************************************************
 * \file NavManager.h
 * \date 2019/01/15 10:11
 *
 * \author wufan
 * Contact: love19862003@163.com
 *
 * \brief navmanager for recast interface
 *
 * TODO: long description
 *
 * \note
*************************************************/
#pragma once
#include <string>
#include "NavCommon.h"
#include "DetourNavMesh.h"

class dtNavMesh;
class dtNavMeshQuery;
class dtQueryFilter;

namespace NavSpace{

  class NavScene;
  class NavManager
  {
  public:
    enum Type{
      _NAV_STATIC_ = 0,
      _NAV_DYNAMIC_ = 1,
    };

    explicit NavManager();

    virtual ~NavManager();

    bool loadNavMesh(const std::string& file);

    bool saveNavMesh(const std::string& file) const;

    bool findPath(const NavPos& s, const NavPos& e, std::vector<NavPos>& res) const;

    bool randomPoint(const NavPos& c, float r, float(*frand)(), NavPos& out) const;

    bool randomPoints(const NavPos& c, int count, float r, float(*frand)(), std::vector<NavPos>& out) const;

    bool rayCast(const NavPos&s, const NavPos& e, NavPos& out) const;

    unsigned short getTileFlag(const NavPos& c) const;

    bool findDistanceToWall(const NavPos&c, float r, float& dis, NavPos& out) const;

    bool isReachAble(const NavPos& p, float diff = 0.001f, NavPos* out = nullptr) const;

  protected:
    dtPolyRef getPolyRef(const NavPos& pos, NavPos* out = nullptr) const;

    bool randomPoint(dtPolyRef& s, const NavPos& c, float r, float(*frand)(), NavPos& out) const;
  protected:
    class dtNavMesh* m_navMesh;
    class dtNavMeshQuery* m_navQuery;
    class dtQueryFilter* m_filter;
  };
}
