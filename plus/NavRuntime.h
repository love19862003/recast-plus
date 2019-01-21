/************************************************
 * \file NavRuntime.h
 * \date 2019/01/21 17:15
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
#include <mutex>
#include <thread>
#include "NavManager.h"
#include "NavScene.h"

namespace NavSpace{

  class NavRuntime : public NavManager, protected NavScene, public NonCopyAble
  {
  public:
    enum NavType {
      STATIC_NAVMESH = 0,
      DYNAMIC_NAMESH = 1,
    };
    explicit NavRuntime(const NotifyCall& notify);

    explicit NavRuntime();

    virtual ~NavRuntime();


    bool start(const std::string& navMesh, const std::string& scene);

    void stop();

    void update();

    virtual bool loadDump(const std::string& path) final;

    virtual bool saveDump(const std::string& path) final;

    bool isDynamicMesh() const{ return DYNAMIC_NAMESH == m_type; }

    void modify(const std::vector<WorldItem>& items, const std::vector<MObjId>& ids);

    bool addMesh(const MeshPtr& mesh);
  protected:
    virtual bool buildTile(const TileIndex& tile) final;

    typedef std::map<TileIndex, TileNavCache> TileCacheMap;
    typedef std::vector<WorldItem> VItems;
    
    void threadMain();
  private:
    TileCacheMap m_cacheMap;
    MObjList m_waitRemove;
    VItems m_waitAdd;
    MObjList m_addDone;
    MObjList m_removeDone;
    std::mutex m_mutex_run;
    std::mutex m_mutex_update;
    std::thread m_thread;
    NotifyCall m_nofity;
    bool m_threadRun;
    const NavType m_type;
    rcContext* m_ctx;
  };
}
