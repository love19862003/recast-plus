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
namespace NavSpace{

  class NavScene
  {
  public:
    typedef Utility::ObjPtrMap<MeshId, Mesh> MeshMap;
    typedef Utility::ObjPtrMap<MObjId, MeshObject> ObjectMap;
    typedef std::pair<int, int> TileIndex;
    
    struct TileNacCache {
      TileIndex index;
      unsigned char* data;
      size_t size;
    };

    TileNacCache buildTile(rcContext* ctx, const TileIndex& tile);
  private:
    unsigned char*  buildTileReal(rcContext* ctx, const TileIndex& tile, const float bmin[3], const float bmax[3], size_t& dataSize);
  protected:
  private:
    ObjectMap m_objects;
    MeshMap m_meshs;
    NavSetting m_setting;
  };

}
