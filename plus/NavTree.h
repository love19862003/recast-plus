/************************************************
 * \file NavTree.h
 * \date 2019/01/14 16:41
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
#include "NavPool.h"
namespace NavSpace{
  
  constexpr size_t treeChildCount(){ return 2; }
  typedef std::array<float, 3> Coor;
  struct MeshBouns{
    MeshBouns(){
      bmin.fill(0.f);
      bmax.fill(0.f);
    }
    Coor bmin;
    Coor bmax;
  };

  struct TreeBouns{
    typedef std::array<float, 2> Bouns;
    Bouns bmin;
    Bouns bmax;

    TreeBouns(){
      bmin.fill(0.f);
      bmax.fill(0.f);
    }

    void fill(const MeshBouns& b){
      bmin[0] = b.bmin[0];
      bmin[1] = b.bmin[2];
      bmax[0] = b.bmax[0];
      bmax[1] = b.bmax[2];
    }

    void meger(const TreeBouns& b){
      bmin[0] = std::min<float>(bmin[0], b.bmin[0]);
      bmin[1] = std::min<float>(bmin[1], b.bmin[1]);
      bmax[0] = std::max<float>(bmax[0], b.bmax[0]);
      bmax[1] = std::max<float>(bmax[1], b.bmax[1]);
    }
  };
  typedef Pool<size_t> TriIdPool;

  struct TreeNode{
    TreeNode():tris(0){
      depth = num = 0;
      leaf = false;
      for (auto& c : child){
        c = nullptr;
      }
    }
    int depth;
    int num;
    bool leaf;
    TreeBouns bouns;
    TriIdPool tris;
    TreeNode* child[treeChildCount()];
  };
}