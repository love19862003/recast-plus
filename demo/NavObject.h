/************************************************
 * \file NavObject.h
 * \date 2019/01/10 18:18
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
#include <array>
#include <memory>
#include "NavPool.h"
#include "NavCommon.h"

struct rcConfig;
class rcContext;
struct rcHeightfield;
struct rcCompactHeightfield;

namespace NavSpace{
  class Mesh;
  typedef std::shared_ptr<Mesh> MeshPtr;
  
  struct TreeNode;
  //typedef std::shared_ptr<TreeNode> TreePtr;

  class MeshObject;
  typedef std::shared_ptr<MeshObject> ObjectPtr;

  struct BuildOffMeshData;
  typedef std::unique_ptr<BuildOffMeshData> BuildOffMeshConPtr;


  typedef Pool<float, 3> VertPool;
  typedef Pool<int, 3> TriPool;
  typedef VertPool NormalPool;
  typedef VertPool::ARRAY Coor;

  struct MeshBouns{
    MeshBouns(){
      bmin.fill(0.f);
      bmax.fill(0.f);
    }
    Coor bmin;
    Coor bmax;
  };
  struct WorldPos{
    Coor m_pos;
    float m_o;
    float m_scale;
    MeshId m_mesh;
  };
  static const size_t MAX_VOLUMES_POINTS = 12;
#pragma pack(push, 1)
  struct OffMeshCon{
    unsigned int id;
    float verts[2*3];
    float rad;
    unsigned char dir;
    unsigned char area;
    unsigned short flag;
  };

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

  struct NavDataBase
  {
    VertPool m_verts;
    TriPool m_tris;
    MeshBouns m_bouns;
    void calcuteBouns();
  };

  class Mesh  : public NavDataBase
  {
  public:
    explicit Mesh(const std::string& file, const MeshId id);
    virtual ~Mesh();

    static MeshPtr loadMesh(const MeshId id, const std::string& file);

    inline const std::string& name() const{ return m_name; }
    inline MeshId id() const{ return m_id; }

  protected:
  protected:
    const std::string m_name;
    const MeshId m_id;
    friend class MeshAction;
  };

  class MeshObject : public  NavDataBase
  {
  public:
    MeshObject(MObjId id, MeshPtr mesh, const WorldPos& pos);
    MeshObject(MObjId id);
    virtual ~MeshObject();
    bool rasterizeTriangles(rcContext* ctx, const rcConfig& cfg, rcHeightfield& solid);
    bool raycastMesh(float* src, float* dst, float& tmin);
    void markVolume(rcContext* ctx, rcCompactHeightfield& chf);

    const float* getOffMeshConnectionVerts() const;
    const float* getOffMeshConnectionRads() const;
    const unsigned char* getOffMeshConnectionDirs() const;
    const unsigned char* getOffMeshConnectionAreas() const;
    const unsigned short* getOffMeshConnectionFlags() const;
    const unsigned int* getOffMeshConnectionId() const;
    int getOffMeshConnectionCount() const;

  protected:
    std::list<TreeNode*> getOverlappingRect(const MeshBouns& bouns);
    std::list<TreeNode*> getOverlappingSegment(const MeshBouns& bouns);
    bool initFromMesh(MeshPtr ptr);
    void calcuteNormals();
    void offMeshConBuild() const;
  private:
    MObjId m_id;
    WorldPos m_pos;
    TreeNode* m_tree;
    size_t m_maxTriPerChunk;
    OffMeshConPool m_offCons;
    VolumePool m_volumes;
    NormalPool m_normals;
    mutable BuildOffMeshConPtr m_buildOff;
    friend class MeshAction;
  };



}
