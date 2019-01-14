/************************************************
 * \file NavVolumeConn.h
 * \date 2019/01/14 16:02
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
#include "NavCommon.h"
#include "NavPool.h"
namespace NavSpace{
#pragma pack(push, 1)
  struct OffMeshCon{
    unsigned int id;
    float verts[2 * 3];
    float rad;
    unsigned char dir;
    unsigned char area;
    unsigned short flag;
  };

  static const size_t MAX_VOLUMES_POINTS = 12;
  struct Volume{
    float verts[MAX_VOLUMES_POINTS * 3];
    float hmin;
    float hmax;
    int nverts;
    int area;
  };

#pragma pack(pop)

  typedef Pool<OffMeshCon> OffMeshConPool;
  typedef Pool<Volume> VolumePool;

  struct VolumeOffCon {
    OffMeshConPool m_offCons;
    VolumePool m_volumes;
  };

  typedef std::shared_ptr<VolumeOffCon> VolOffConPtr;
}
