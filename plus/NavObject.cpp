/************************************************
 * \file NavObject.cpp
 * \date 2019/01/10 19:16
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

#include <assert.h>
#include "NavObject.h"
#include "NavMatrix.h"
#include "MeshLoaderObj.h"
#include "DetourCommon.h"
#include "Recast.h"
#include "NavResource.h"

namespace NavSpace{

  static void backVisit(TreeNode* node, const std::function<void(TreeNode*)>& fun){
    if (!node || !fun) return;
    for (auto& c : node->child){
      backVisit(c, fun);
    }
    fun(node);
  }

  static void checkVisit(TreeNode* node, const std::function<bool(TreeNode*)>& fun){
    if (!node || !fun) return;
    if (fun(node)){
      for (auto& c : node->child){
        checkVisit(c, fun);
      }
    }
  }

  static bool checkAssert(TreeNode* node,  const std::function<bool(TreeNode*)>& fun){
    if (!node || !fun){
      return false;
    }

    bool r = fun(node);
    for (auto& c : node->child){
      if (c && checkAssert(c, fun)){
        assert(r);
        break;
      }
    }
    return r;
  }

  

  static void calMeshBouns(const float* arr, MeshBouns& bouns, bool fource){
    for (size_t i = 0; i < 3; ++i){
      if (fource || bouns.bmin[i] > arr[i]){
        bouns.bmin[i] = arr[i];
      }
      if (fource || bouns.bmax[i] < arr[i]){
        bouns.bmax[i] = arr[i];
      }
    }
  }

  static void calTreeBouns(const float* v, TreeBouns& bouns, bool fource){
    if (fource || bouns.bmin[0] > v[0]){
      bouns.bmin[0] = v[0];
    }
    if (fource || bouns.bmax[0] < v[0]){
      bouns.bmax[0] = v[0];
    }

    if (fource || bouns.bmin[1] > v[2]){
      bouns.bmin[1] = v[2];
    }
    if (fource || bouns.bmax[1] < v[2]){
      bouns.bmax[1] = v[2];
    }
  }
  
  void NavDataBase::calcuteBouns(){
    m_verts.call([this](const float* v, size_t index){
      calMeshBouns(v, m_bouns, index == 0);
    });
    assert(m_bouns.bmax[0] > m_bouns.bmin[0]);
    assert(m_bouns.bmax[1] > m_bouns.bmin[1]);
    assert(m_bouns.bmax[2] > m_bouns.bmin[2]);
  }


  Mesh::Mesh(const std::string& file, const MeshId id): NavDataBase(), m_name(file), m_id(id){

  }
  
  Mesh::~Mesh(){

  }
  static const std::string MESH_OBJ_TAG = ".obj";
  MeshPtr Mesh::loadMesh(const MeshId id, const std::string& file){
    auto pos = file.find_last_of(".");
    if (pos == std::string::npos){
      return nullptr;
    }

    auto tag = file.substr(pos);
    if (tag == MESH_OBJ_TAG){
      rcMeshLoaderObj loader;
      if (!loader.load(file)){
        return nullptr;
      }

      auto res = std::make_shared<Mesh>(file, id);
      res->m_verts.add(loader.getVerts(), loader.getVertCount());
      res->m_tris.add(loader.getTris(), loader.getTriCount());
      res->calcuteBouns();
      return res;
    }
    return NavResource::readMesh(file, id);
  }

  bool Mesh::saveMesh(const std::string& path){
    bool res = NavResource::writeMesh(path, *this);
    assert(res);
    return res;
  }

  MeshObject ::MeshObject(MObjId id, MeshPtr mesh, const WorldItem& pos) : NavDataBase(), m_id(id), m_pos(pos){
    assert(m_pos.m_mesh == mesh->id());
    assert(mesh);
    m_tree = nullptr;
    m_maxTriPerChunk = 0;
    initFromMesh(mesh);
  }

  MeshObject::MeshObject(MObjId id) : NavDataBase(), m_id(id){
    m_pos.m_mesh = INVALID_MESH_ID;
    m_pos.m_o = 0.f;
    m_pos.m_pos.fill(0.f);
    m_pos.m_scale = 1.0f;
    m_tree = nullptr;
    m_maxTriPerChunk = 0;
  }
  
  MeshObject::~MeshObject(){
    backVisit(m_tree, [](TreeNode* node){ delete node; node = nullptr; });
    m_tree = nullptr;
  }

  struct BoundsItem{
    size_t tri;
    TreeBouns bouns;
  };

  static void calExtends(const BoundsItem& item, TreeNode* node, bool fource){
    if(fource){
      node->bouns.copy(item.bouns);
    }else{
      node->bouns.meger(item.bouns);
    }
  }

  static void calExtends(BoundsItem* items, int min, int max, TreeNode* node){
    for (int i = min; i < max; ++i){
      calExtends(items[i], node, i== min);
    }
  }

  static bool compareXX(const BoundsItem& l, const BoundsItem& r){
    return l.bouns.bmin[0] > r.bouns.bmin[0];
  }
  static bool compareYY(const BoundsItem& l, const BoundsItem& r){
    return l.bouns.bmin[1] > r.bouns.bmin[1]; 
  }
  static void subdivide(BoundsItem* items, int nitems, int min, int max, int&depth, TreeNode* node, size_t& maxTriPerChunk){
    if (!node) return;
    int num = max - min;
    node->num = num;
    node->depth = ++depth;
    const int per = 256;
    if (num < per){
      node->leaf = true;
      calExtends(items, min, max, node);
      node->tris.resize(num);
      for (int i = min; i < max; ++i){
        node->tris.add(&items[i].tri, 1);
      }
      if (num > maxTriPerChunk){
        maxTriPerChunk = num;
      }
      assert(maxTriPerChunk <= per);
    }else{
      node->leaf = false;
      calExtends(items, min, max, node);
      auto& b = node->bouns;
      if (node->bouns.logxy()){
        std::sort(items + min, items + max, compareXX);
      }else{
        std::sort(items + min, items + max, compareYY);
      }

      int nodeCount = num / treeChildCount();
      for (int i = 0; i < treeChildCount(); ++i){
        depth = node->depth;
        node->child[i] = new TreeNode;
        int is = min + i * nodeCount;
        int ie = is + nodeCount;
        if(i == treeChildCount() - 1){ ie = max;}
        subdivide(items, nitems, is, ie, depth, node->child[i], maxTriPerChunk);
      }
    }
  }

  static void calNormal(float* out, const float* v0, const float* v1, const float* v2){
    float e0[3], e1[3];
    for (int j = 0; j < 3; ++j){
      e0[j] = v1[j] - v0[j];
      e1[j] = v2[j] - v0[j];
    }
    out[0] = e0[1] * e1[2] - e0[2] * e1[1];
    out[1] = e0[2] * e1[0] - e0[0] * e1[2];
    out[2] = e0[0] * e1[1] - e0[1] * e1[0];
    float d = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    if (d > 0){
      d = 1.0f / d;
      out[0] *= d;
      out[1] *= d;
      out[2] *= d;
    }
  }

  bool MeshObject::initFromMesh(MeshPtr ptr){
    if (!ptr) return false;
    m_verts.resize(ptr->m_verts.count());
    m_verts.add(ptr->m_verts);
    m_tris.resize(ptr->m_tris.count());
    m_tris.add(ptr->m_tris);
   

    //计算世界坐标
    m_verts.call([this](float*v, size_t index){
      float in[3];
      dtVcopy(in, v);
      Matirx::matirx(v, in, m_pos);
      assert(m_id != INVALID_MOBJ_ID || dtVequal(v, in));
    });
    initVolumeOffConn();
    //计算包围盒
    calcuteBouns();
    size_t ntri = m_tris.count();
    BoundsItem* items = new BoundsItem[ntri];
    if (!items) return false;
    m_tris.call([&items, this](const int* tri, size_t index){
      items[index].tri = index;
      assert(tri[0] < m_verts.count());
      assert(tri[1] < m_verts.count());
      assert(tri[2] < m_verts.count());
      const float* v0 = m_verts.pool(tri[0]);
      const float* v1 = m_verts.pool(tri[1]);
      const float* v2 = m_verts.pool(tri[2]);
      calTreeBouns(v0, items[index].bouns, true);
      calTreeBouns(v1, items[index].bouns, false);
      calTreeBouns(v2, items[index].bouns, false);

      //cal tri normal
      float nor[3];
      calNormal(nor, v0, v1, v2);
      m_normals.add(nor, 1);
    });
    

    int depth = 0;
    m_tree = new TreeNode;
    if (!m_tree) return false;
    m_tree->bouns.fill(m_bouns);
    subdivide(items, ntri, 0, ntri, depth, m_tree, m_maxTriPerChunk);
    delete[] items;
    return true;
  }

  void MeshObject::calcuteNormals(){
    m_normals.resize(m_tris.count());
    m_normals.reset();
    m_tris.call([this](const int* tri, size_t index){
      assert(tri[0] < m_verts.count());
      assert(tri[1] < m_verts.count());
      assert(tri[2] < m_verts.count());
      const float* v0 = m_verts.pool(tri[0]);
      const float* v1 = m_verts.pool(tri[1]);
      const float* v2 = m_verts.pool(tri[2]);
      float nor[3];
      calNormal(nor, v0, v1, v2);
      m_normals.add(nor, 1);
    });
  }

  void MeshObject::initVolumeOffConn(){

    m_volumeOffConn.m_offCons.call([this](OffMeshCon* conn, size_t index){
      float s[3], e[3];
      Matirx::matirx(s, conn->verts, m_pos);
      Matirx::matirx(s, conn->verts+ 3, m_pos);
      dtVcopy(conn->verts, s);
      dtVcopy(conn->verts + 3, e);
    });

    m_volumeOffConn.m_volumes.call([this](Volume* vol, size_t index){
      for (int i = 0 ; i < vol->nverts; ++i){
        float s[3];
        Matirx::matirx(s, vol->verts + i * 3, m_pos);
        dtVcopy(vol->verts + i * 3, s);
      }
      vol->hmin *= m_pos.m_scale;
      vol->hmax *= m_pos.m_scale;
    });

  }

  static bool checkOverlapSegment(const TreeBouns& t, const TreeBouns& b){
    static const float EPSILON = 1e-6f;

    float tmin = 0;
    float tmax = 1;
    float d[2];
    d[0] = t.bmax[0] - t.bmin[0];
    d[1] = t.bmax[1] - t.bmin[1];

    for (int i = 0; i < 2; i++){
      if (fabsf(d[i]) < EPSILON){
        // Ray is parallel to slab. No hit if origin not within slab
        if (t.bmin[i] < b.bmin[i] || t.bmin[i] >b.bmax[i])
          return false;
      } else{
        // Compute intersection t value of ray with near and far plane of slab
        float ood = 1.0f / d[i];
        float t1 = (b.bmin[i] - t.bmin[i]) * ood;
        float t2 = (b.bmax[i] - t.bmin[i]) * ood;
        if (t1 > t2){ float tmp = t1; t1 = t2; t2 = tmp; }
        if (t1 > tmin) tmin = t1;
        if (t2 < tmax) tmax = t2;
        if (tmin > tmax) return false;
      }
    }
    return true;
  }

  static bool checkOverlapRect(const  TreeBouns& a, const TreeBouns& b){
    bool overlap = true;
    overlap = (a.bmin[0] > b.bmax[0] || a.bmax[0] < b.bmin[0]) ? false : overlap;
    overlap = (a.bmin[1] > b.bmax[1] || a.bmax[1] < b.bmin[1]) ? false : overlap;
    return overlap;
  }

  std::list<TreeNode*>  MeshObject::getOverlappingRect(const MeshBouns& bouns){
    TreeBouns b;
    b.fill(bouns);
    std::list<TreeNode*> res;
    checkVisit(m_tree, [&b, &res](TreeNode* node){
      const bool overlap = checkOverlapRect(b, node->bouns);
      if (overlap && node->leaf){
        res.push_back(node);
      }
      return true;
    });
    return res;
  }

  std::list<TreeNode*> MeshObject::getOverlappingSegment(const MeshBouns& bouns){
    TreeBouns b;
    b.fill(bouns);
    std::list<TreeNode*> res;
    checkVisit(m_tree, [&b,&res](TreeNode* node){
      const bool overlap = checkOverlapSegment(b, node->bouns);
      if (overlap && node->leaf){
        res.push_back(node);
      }
      return overlap;
    });
    return res;
  }

  bool MeshObject::rasterizeTriangles(rcContext* ctx, const rcConfig& cfg, rcHeightfield& solid){
    bool res = true;
    MeshBouns bouns;
    dtVcopy(bouns.bmin.data(), cfg.bmin);
    dtVcopy(bouns.bmax.data(), cfg.bmax);
    unsigned char* triareas = new unsigned char[m_maxTriPerChunk];
    TriPool pool;
    pool.resize(m_maxTriPerChunk);
    auto list = getOverlappingRect(bouns);
    for (auto& node : list){
      assert(node->leaf);
      assert(node->num > 0); 
      pool.reset();
      node->tris.call([&pool, this](const size_t* idx, size_t){
        size_t triIndex = *idx;
        assert(triIndex < m_tris.count());
        const int* tri = m_tris.pool(triIndex);
        pool.add(tri);
      });
      assert(pool.count() <= m_maxTriPerChunk);
      memset(triareas, 0, pool.count() * sizeof(unsigned char));
      rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, m_verts.pool(), m_verts.count(), pool.pool(), pool.count(), triareas);
      if (!rcRasterizeTriangles(ctx, m_verts.pool(), m_verts.count(), pool.pool(), triareas, pool.count(), solid, cfg.walkableClimb)){
        res = false;
        break;
      }
    }
    delete[] triareas;
    return res;
  }

  static bool isectSegAABB(const float* sp, const float* sq,const float* amin, const float* amax,float& tmin, float& tmax){
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

  static bool intersectSegmentTriangle(const float* sp, const float* sq,  const float* a, const float* b, const float* c, float &t){
    float v, w;
    float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
    rcVsub(ab, b, a);
    rcVsub(ac, c, a);
    rcVsub(qp, sp, sq);

    // Compute triangle normal. Can be precalculated or cached if
    // intersecting multiple segments against the same triangle
    rcVcross(norm, ab, ac);

    // Compute denominator d. If d <= 0, segment is parallel to or points
    // away from triangle, so exit early
    float d = rcVdot(qp, norm);
    if (d <= 0.0f) return false;

    // Compute intersection t value of pq with plane of triangle. A ray
    // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
    // dividing by d until intersection has been found to pierce triangle
    rcVsub(ap, sp, a);
    t = rcVdot(ap, norm);
    if (t < 0.0f) return false;
    if (t > d) return false; // For segment; exclude this code line for a ray test

                             // Compute barycentric coordinate components and test if within bounds
    rcVcross(e, qp, ap);
    v = rcVdot(ac, e);
    if (v < 0.0f || v > d) return false;
    w = -rcVdot(ab, e);
    if (w < 0.0f || v + w > d) return false;

    // Segment/ray intersects triangle. Perform delayed division
    t /= d;

    return true;
  }

  bool MeshObject::raycastMesh(float* src, float* dst, float& tmin){
    float dir[3];
    rcVsub(dir, dst, src);

    // Prune hit ray.
    float btmin, btmax;
    if (!isectSegAABB(src, dst, m_bouns.bmax.data(), m_bouns.bmin.data(), btmin, btmax))
      return false;

    float delay[3];
    rcVsub(delay, dst, src);

    MeshBouns b;
    dtVmad(b.bmin.data(), src, delay, btmin);
    dtVmad(b.bmax.data(), src, delay, btmax);
   
    auto list = getOverlappingSegment(b);
    if (list.empty()){
      return false;
    }

    tmin = 1.f;
    bool hit = false;
    for (auto& node : list){
      assert(node->leaf);
      node->tris.call([this, &hit, &tmin, src, dst](const size_t* idx, size_t){
        size_t triIndex = *idx;
        assert(triIndex < m_tris.count());
        const int* tri = m_tris.pool(triIndex);
        const float* v0 = m_verts.pool(tri[0]);
        const float* v1 = m_verts.pool(tri[1]);
        const float* v2 = m_verts.pool(tri[2]);
        float t = 1.f;
        if (intersectSegmentTriangle(src, dst, v0,  v1, v2, t)){
          hit = true;
          if (t < tmin){ tmin = t;}
        }
      });
    }
    return hit;
  }


  void MeshObject::markVolume(rcContext* ctx, rcCompactHeightfield& chf){
    m_volumeOffConn.m_volumes.call([&](const Volume* v, size_t index){
      rcMarkConvexPolyArea(ctx, v->verts, v->nverts, v->hmin, v->hmax, (unsigned char)v->area, chf);
    });
  }

  bool MeshObject::saveMap(const std::string& name){
    return NavResource::writeObject(name, *this);
  }
}
