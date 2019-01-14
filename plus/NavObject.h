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
#include "NavTree.h"
#include "NavVolumeConn.h"
#include "NavMatrix.h"


struct rcConfig;
class rcContext;
struct rcHeightfield;
struct rcCompactHeightfield;
namespace NavSpace{
  class Mesh;
  typedef std::shared_ptr<Mesh> MeshPtr;
  class MeshObject;
  typedef std::shared_ptr<MeshObject> ObjectPtr;

  typedef Pool<float, 3> VertPool;
  typedef Pool<int, 3> TriPool;
  typedef VertPool NormalPool;

  struct NavDataBase{
    VertPool m_verts;
    TriPool m_tris;
    MeshBouns m_bouns;
    void calcuteBouns();
  };

  class Mesh : public NavDataBase , public NonCopyAble{
  public:
    explicit Mesh(const std::string& file, const MeshId id);
    virtual ~Mesh();

    static MeshPtr loadMesh(const MeshId id, const std::string& file);

    inline const std::string& name() const{ return m_name; }

    inline MeshId id() const{ return m_id; }

    bool saveMesh(const std::string& path);
  protected:
  protected:
    const std::string m_name;
    const MeshId m_id;
    friend class NavResource;
  };

  class MeshObject : public  NavDataBase , public NonCopyAble{
  public:
    explicit MeshObject(MObjId id, MeshPtr mesh, const WorldPos& pos);
    virtual ~MeshObject();
    bool rasterizeTriangles(rcContext* ctx, const rcConfig& cfg, rcHeightfield& solid);
    bool raycastMesh(float* src, float* dst, float& tmin);
    void markVolume(rcContext* ctx, rcCompactHeightfield& chf);
    bool saveMap(const std::string& name);
    const VolumeOffCon& volumeOffConns() const{ return m_volumeOffConn; }

  private:
    explicit MeshObject(MObjId id);
  protected:
    std::list<TreeNode*> getOverlappingRect(const MeshBouns& bouns);
    std::list<TreeNode*> getOverlappingSegment(const MeshBouns& bouns);
    bool initFromMesh(MeshPtr ptr);
    void calcuteNormals();
    void initVolumeOffConn();
  private:
    MObjId m_id;
    WorldPos m_pos;
    TreeNode* m_tree;
    size_t m_maxTriPerChunk;
    VolumeOffCon m_volumeOffConn;
    NormalPool m_normals;
    friend class NavResource;   
  };


}
