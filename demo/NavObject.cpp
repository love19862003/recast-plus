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
#include <fstream>
#include <assert.h>
#include "NavObject.h"
#include "MeshLoaderObj.h"
#include "DetourCommon.h"
#include "Recast.h"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/operation.hpp>

namespace NavSpace{

  static void rotate(float* out, const float in[3], const WorldPos& w){
    namespace ublas = boost::numeric::ublas ;
    ublas::vector<float> vec;
    ublas::matrix<float> m(4, 4, 0.f);
    vec(0) = in[0];
    vec(1) = in[1];
    vec(2) = in[2];
    vec(3) = 1.f;

    const static float pi = 3.14159265358f;
    float o = w.m_o / 180.f;
    m(0, 0) = std::cos(o);
    m(0, 2) = -std::sin(o);
    m(1, 1) = 1.f;

    m(2, 0) = std::sin(o);
    m(2, 2) = std::cos(o);
    m(3, 3) = 1.f;

    ublas::vector<float> res(4, 0.f);
    ublas::axpy_prod(vec, m, res);
    float r[3];
    r[0] = res(0);
    r[1] = res(1);
    r[2] = res(2);
    dtVmad(out, w.m_pos.data(), r, w.m_scale);
  }

  constexpr size_t treeChildCount(){ return 2; }
  static const unsigned int MESH_MAGIC = '1' << 24 | '0' << 16 | '0' << 8 | '0';
  static const unsigned int MAP_MAGIC = '9' << 24 | '0' << 16 | '0' << 8 | '0';
  static const std::string MESH_TAG = ".mesh";
  static const std::string MESH_OBJ_TAG = ".obj";
  static const std::string MAP_TAG = ".map";

  struct TreeBouns{
    typedef std::array<float, 2> Bouns;
    Bouns bmin;
    Bouns bmax;

    TreeBouns() {
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

  static void calMeshBouns(const VertPool::ARRAY& arr, MeshBouns& bouns, bool fource){
    if (fource){
      bouns.bmin = arr;
      bouns.bmax = arr;
    } else{
      for (size_t i = 0; i < arr.size(); ++i){
        if (bouns.bmin[i] >  arr[i]){
          bouns.bmin[i] = arr[i];
        }
        if (bouns.bmax[i] < arr[i]){
          bouns.bmin[i] = arr[i];
        }
      }
    }
  }

  static void calMeshBouns(const float* v, MeshBouns& bouns, bool fource){
    VertPool::ARRAY arr;
    for (size_t i = 0; i < arr.size(); ++i){ 
      arr[i] = v[i];
    }
    calMeshBouns(arr, bouns, fource);
  }

  static void calTreeBouns(const VertPool::ARRAY& arr, TreeBouns& bouns, bool fource){
    MeshBouns b;
    calMeshBouns(arr, b, fource);
    bouns.fill(b);
  }

  static void calTreeBouns(const float* v, TreeBouns& bouns, bool fource){
    MeshBouns b;
    calMeshBouns(v, b, fource);
    bouns.fill(b);
  }
  
  void NavDataBase::calcuteBouns(){
    m_verts.call([this](const float* v, size_t index){
      calMeshBouns(v, m_bouns, index == 0);
    });
  }

#pragma pack(push, 1)
  struct DataBaseHeader{
    unsigned int version;
    unsigned int vers;  
    unsigned int tris;  
  };

  struct DataMapHeader{
    unsigned int version;
    unsigned int volmes;
    unsigned int offcons;
    unsigned int tree;
  };

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
#pragma pack(pop)

  class MeshAction
  {
  public:
    static MeshPtr readMesh(const std::string& file, MeshId id){
      auto res = std::make_shared<Mesh>(file, id);
      if (!res) return nullptr;
      try{
        std::ifstream ifile(file, std::ios::in | std::ios::binary);
        if (!readBase(ifile, *res)){
          return nullptr;
        }

        ifile.close();
        return res;

      }catch (...){
        return nullptr;
      }
    }
    static bool writeMesh(const std::string& file, MeshPtr ptr){
      if (!ptr){ return false; }
      try{
        std::ofstream ofile(file, std::ios::out | std::ios::binary);
        writeBase(ofile, *ptr);
        ofile.close();
        return true;
      } catch (...){
        return false;
      }
    }

    static ObjectPtr readObject(const std::string& file){
      ObjectPtr res = std::make_shared<MeshObject>(INVALID_MOBJ_ID);
      try{
        std::ifstream ifile(file, std::ios::in | std::ios::binary);
        if (!readBase(ifile, *res)){
          return nullptr;
        }

        DataMapHeader header;
        ifile.read((char*)&header, sizeof(DataMapHeader));
        readPool(ifile, header.offcons, res->m_offCons);
        readPool(ifile, header.volmes, res->m_volumes);
        if(header.tree){ res->m_tree = readNode(ifile, res->m_maxTriPerChunk); }
        res->calcuteNormals();
        ifile.close();
      } catch (...){
        return nullptr;
      }
    }

    static bool writeObject(const std::string& file, ObjectPtr obj){
      if (!obj){ return false; }
      try{
        std::ofstream ofile(file, std::ios::out | std::ios::binary);
        writeBase(ofile, *obj);
        DataMapHeader header;
        header.version = MAP_MAGIC;
        header.offcons = obj->m_offCons.count();
        header.volmes = obj->m_volumes.count();
        header.tree = obj->m_tree ? 1 : 0;
        if (header.tree > 0){
          writeNode(ofile, obj->m_tree);
        }
        
        ofile.close();
        return true;
      } catch (...){
        return false;
      }
    }
  protected:
    static TreeNode*  readNode(std::ifstream& ifile, size_t& maxTri){
      TreeNode* node = new TreeNode;
      ifile.read((char*)&node->depth, sizeof(int));
      ifile.read((char*)&node->num, sizeof(int));
      ifile.read((char*)&node->leaf, sizeof(bool));
      ifile.read((char*)node->bouns.bmin.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmin.size());
      ifile.read((char*)node->bouns.bmax.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmax.size());
      size_t c = 0;
      ifile.read((char*)&c, sizeof(size_t));
      if(c > 0){
        readPool(ifile, c, node->tris);
      }

      if (c > maxTri){ maxTri = c;}

      for (size_t i = 0; i < treeChildCount(); ++i){
        if(node->leaf){
          node->child[i] = nullptr;
        }else{
          node->child[i] = readNode(ifile, maxTri);
        }
      }
      return node;
    }
    static void writeNode(std::ofstream& ofile, TreeNode* node){
      if (!node) return;
      ofile.write((const char*)&node->depth, sizeof(int));
      ofile.write((const char*)&node->num, sizeof(int));
      ofile.write((const char*)&node->leaf, sizeof(bool));
      ofile.write((const char*)node->bouns.bmin.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmin.size());
      ofile.write((const char*)node->bouns.bmax.data(), sizeof(TreeBouns::Bouns::value_type) * node->bouns.bmax.size());
      size_t c = node->tris.count();
      ofile.write((const char*)&c, sizeof(size_t));
      if (c > 0){
        writePool(ofile, node->tris);
      }
      if (!node->leaf){
        for (size_t i = 0; i < treeChildCount(); ++i){
          writeNode(ofile, node->child[i]);
        }
      }
    }
    static bool readBase(std::ifstream& ifile, NavDataBase& data){
      DataBaseHeader head;
      ifile.read((char*)&head, sizeof(DataBaseHeader));
      if (head.version != MESH_MAGIC){ return false; }
      readPool(ifile, head.vers, data.m_verts);
      readPool(ifile, head.tris, data.m_tris);
      data.calcuteBouns();
      return true;
    }
    static bool writeBase(std::ofstream& ofile, const NavDataBase& data){
      DataBaseHeader head;
      head.version = MESH_MAGIC;
      head.vers = data.m_verts.count();
      head.tris = data.m_tris.count();
      ofile.write((const char*)&head, sizeof(DataBaseHeader));
      writePool(ofile, data.m_verts);
      writePool(ofile, data.m_tris);
      return true;
    }

    template<typename T>
    static void readPool(std::ifstream& ifile, size_t count, T& pool){
      pool.resize(count + 1);
      for (size_t i = 0; i < count; ++i){
        T::ARRAY arr;
        ifile.read((char*)arr.data(), T::ObjectSize());
        pool.add(arr.data(), 1);
      }
    } 

    template<typename T>
    static void writePool(std::ofstream& ofile, const T& pool){
      ofile.write((const char*)pool.pool(), pool.count() * T::ObjectSize());
    }

  protected:
  private:
  };

  Mesh::Mesh(const std::string& file, const MeshId id): NavDataBase(), m_name(file), m_id(id){

  }
  
  Mesh::~Mesh(){

  }

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

    if (tag == MESH_TAG){
      return  MeshAction::readMesh(file, id);
    }
    return nullptr;
  }

  MeshObject ::MeshObject(MObjId id, MeshPtr mesh, const WorldPos& pos) : NavDataBase(), m_id(id), m_pos(pos){
    assert(m_pos.m_mesh == mesh->id());
    assert(mesh);
    m_tree = nullptr;
    initFromMesh(mesh);
  }
  
  MeshObject::MeshObject(MObjId id) : NavDataBase(), m_id(id){
    m_pos.m_mesh = INVALID_MESH_ID;
    m_pos.m_o = 0.f;
    m_pos.m_pos.fill(0.f);
    m_pos.m_scale = 1.0f;
    m_tree = nullptr;
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
      node->bouns = item.bouns;
    }else{
      node->bouns.meger(item.bouns);
    }
  }

  static void calExtends(BoundsItem* items, int min, int max, TreeNode* node){
    for (int i = min; i < max; ++i){
      calExtends(items[i], node, i== min);
    }
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
    }else{
      node->leaf = false;
      calExtends(items, min, max, node);
      auto& b = node->bouns;
      if (b.bmax[0] - b.bmin[0] > b.bmax[1] - b.bmin[1]){
        std::sort(items + min, items + max, [](const BoundsItem& l, const BoundsItem& r){return l.bouns.bmin[0] > r.bouns.bmin[0]; });
      }else{
        std::sort(items + min, items + max, [](const BoundsItem& l, const BoundsItem& r){return l.bouns.bmin[1] > r.bouns.bmin[1]; });
      }

      int nodeCount = num / treeChildCount();
      for (int i = 0; i < nodeCount; ++i){
        depth = node->depth;
        node->child[i] = new TreeNode;
        int is = min + i * nodeCount;
        int ie = is + nodeCount;
        if(i == treeChildCount() - 1){ ie = max;}
        subdivide(items, nitems, is, ie, depth, node, maxTriPerChunk);
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
      rotate(v, v, m_pos);
    });
    //计算包围盒
    calcuteBouns();
    size_t ntri = m_tris.count();
    BoundsItem* items = new BoundsItem[ntri];
    if (!items) return false;
    m_tris.call([items, this](const int* tri, size_t index){
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

      float nor[3];
      calNormal(nor, v0, v1, v2);
      m_normals.add(nor, 1);
    });

    int depth = 0;
    m_tree = new TreeNode;
    if (!m_tree) return false;
    subdivide(items, ntri, 0, ntri, depth, m_tree, m_maxTriPerChunk);
    delete[] items;

    calcuteNormals();
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
      if (overlap){
        if (node->leaf){ res.push_back(node); }
        return true;
      } else{
        return false;
      }
    });
    return res;
  }

  std::list<TreeNode*> MeshObject::getOverlappingSegment(const MeshBouns& bouns){
    TreeBouns b;
    b.fill(bouns);
    std::list<TreeNode*> res;
    checkVisit(m_tree, [&b,&res](TreeNode* node){
      const bool overlap = checkOverlapSegment(b, node->bouns);
      if (overlap){
        if (node->leaf){ res.push_back(node); }
        return true;
      }else{
        return false;
      }
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

      if (!node->leaf || node->num <= 0){
        continue;
      }
      pool.reset();
      pool.resize(node->tris.count());
      node->tris.call([&pool, this](const size_t* tri, size_t){
        pool.add(m_tris.pool(*tri), 1);
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

    float p[3], q[3];
    

    MeshBouns b;
    dtVmad(b.bmin.data(), src, delay, btmin);
    dtVmad(b.bmax.data(), src, delay, btmax);
   
    auto list = getOverlappingSegment(b);
    if (list.empty()){
      return false;
    }

    TriPool pool;
    pool.resize(m_maxTriPerChunk);
    bool hit = false;
    const float* vert = m_verts.pool(0);
    for (auto& node : list){
      if (!node->leaf){
        continue;
      }
      pool.reset();
      pool.resize(node->tris.count());
      node->tris.call([&pool, this, &hit, &tmin, src, dst](const size_t* id, size_t){
        pool.add(m_tris.pool(*id), 1);
        const int* tri = m_tris.pool(*id);
        const float* v0 = m_verts.pool(tri[0]);
        const float* v1 = m_verts.pool(tri[1]);
        const float* v2 = m_verts.pool(tri[2]);
        float t = 1.f;
        if (intersectSegmentTriangle(src, dst, v0,  v1, v2, t)){
          hit = true;
          if (t < tmin){ tmin = t;}
        }
      });

    
      return hit;
    }

  }


  void MeshObject::markVolume(rcContext* ctx, rcCompactHeightfield& chf){
    m_volumes.call([&](const Volume* v, size_t index){
      rcMarkConvexPolyArea(ctx, v->verts, v->nverts, v->hmin, v->hmax, (unsigned char)v->area, chf);
    });
  }

  void MeshObject::offMeshConBuild() const{
    if (!m_buildOff){
      m_buildOff.reset(new BuildOffMeshData);
    }
    if (!m_buildOff){ return; }
    m_buildOff->count = m_offCons.count();
    m_buildOff->verts.reset();
    m_buildOff->rads.reset();
    m_buildOff->ids.reset();
    m_buildOff->dirs.reset();
    m_buildOff->areas.reset();
    m_buildOff->flags.reset();
    m_buildOff->dirs.reset();

    m_offCons.call([this](const OffMeshCon* conn, size_t index){
      m_buildOff->verts.add(conn->verts);
      m_buildOff->rads.add(&conn->rad);
      m_buildOff->ids.add(&conn->id);
      m_buildOff->dirs.add(&conn->dir);
      m_buildOff->areas.add(&conn->area);
      m_buildOff->flags.add(&conn->flag);
      m_buildOff->dirs.add(&conn->dir);
    });
  }

  const float* MeshObject::getOffMeshConnectionVerts() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->verts.pool();
  }
  const float* MeshObject::getOffMeshConnectionRads() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->rads.pool();
  }
  const unsigned char* MeshObject::getOffMeshConnectionDirs() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->dirs.pool();
  }
  const unsigned char* MeshObject::getOffMeshConnectionAreas() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->areas.pool();
  
  }
  const unsigned short* MeshObject::getOffMeshConnectionFlags() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->flags.pool();
  
  }
  const unsigned int* MeshObject::getOffMeshConnectionId() const{
    if (!m_buildOff){
      offMeshConBuild();
    }
    return m_buildOff->ids.pool();
  }

  int MeshObject::getOffMeshConnectionCount() const{
    assert( m_buildOff->count == m_offCons.count());
    if (m_buildOff->count != m_offCons.count()){
      offMeshConBuild();
    }
    return m_buildOff->count;
  }


}
