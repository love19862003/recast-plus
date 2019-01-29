/************************************************
 * \file NavRuntime.cpp
 * \date 2019/01/21 17:33
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
#include <set>
#include <iostream>
#include "NavRuntime.h"
#include <Recast.h>
namespace NavSpace{

  typedef std::chrono::system_clock time_clock;
  typedef std::chrono::microseconds ms_type;
  typedef std::chrono::hours hour_type;

  template<typename T>
  inline std::string concat(T t){
    return  std::to_string(t);
  }

  template<>
  inline std::string concat<char*>(char* t){
    return std::string(t);
  }

  template<>
  inline std::string concat<const char*>(const char* t){
    return std::string(t);
  }

  template<>
  inline std::string concat< std::string>(std::string t){
    return t;
  }

  template<typename T, typename ... ARGS>
  inline std::string concat(T t, ARGS ...  args){
    return concat(t) + concat(args ...);
  }

  class RunTimeCtx : public rcContext
  {
  public:
    template<typename T, typename ... ARGS>
    void slog(const rcLogCategory c, T t, ARGS... args){
      auto str = concat(t, args ...);
      sdoLog(c, str);
    }

  protected:
    virtual void doLog(const rcLogCategory c , const char*  str, const int size ) final{
      sdoLog(c, std::string(str, size));
    }
    void sdoLog(const rcLogCategory c, const std::string& str){
      std::cout << str << std::endl;
    }

   

    virtual void doResetTimers() final{
      for (int i = RC_TIMER_TOTAL; i < RC_MAX_TIMERS; ++i){
        _stimers[i] = _etimers[i] = time_clock::now();
      }
    }

    virtual void doStartTimer(const rcTimerLabel lab) final{
      if(lab < RC_MAX_TIMERS){
        _stimers[lab] = time_clock::now();
      }
    }

    virtual void doStopTimer(const rcTimerLabel lab) final{
      if (lab < RC_MAX_TIMERS){
        _etimers[lab] = time_clock::now();
      }
    }

    virtual int doGetAccumulatedTime(const rcTimerLabel lab) const final{
      if (lab < RC_MAX_TIMERS){
        auto delay =  std::chrono::duration_cast<ms_type>(_etimers[lab] - _stimers[lab]).count();
        return delay > 0 ? delay : -1;
      }else{
        return -1;
      }
    }
  private:
    time_clock::time_point _stimers[RC_MAX_TIMERS];
    time_clock::time_point _etimers[RC_MAX_TIMERS];
  };


  NavRuntime::NavRuntime(const NotifyCall& notify) :  NavManager(), NavScene(),m_nofity(notify), m_threadRun(false), m_type(DYNAMIC_NAMESH), m_ctx(new RunTimeCtx){
    assert(nullptr != m_nofity);
  }
  NavRuntime::NavRuntime() : NavManager(), NavScene(), m_nofity(nullptr), m_threadRun(false), m_type(STATIC_NAVMESH), m_ctx(new RunTimeCtx){

  }
  NavRuntime::~NavRuntime(){
    if (isDynamicMesh() && m_threadRun){
      stop();
    }

    if (m_ctx){
      delete m_ctx;
      m_ctx = nullptr;
    }
  }

  bool NavRuntime::start(const std::string& navMesh, const std::string& scene){
    if (!loadNavMesh(navMesh)){
      return false;
    }
    if (!m_navMesh || !m_navQuery || !m_filter ){
      return false;
    }

    if (isDynamicMesh()){
      if (!setScene(scene)){ return false; }
      assert(m_meshs.size() > 0);
      assert(!m_threadRun);
      if (m_threadRun){ return false; }
      m_threadRun = true;
      m_thread = std::move(std::thread(&NavRuntime::threadMain, this));
    }
    return true;
  }

  void NavRuntime::stop(){
    if (isDynamicMesh()){
      m_threadRun = false;
      m_thread.join();
    }
  }

  void NavRuntime::update(){
    if (!m_navMesh || !m_navQuery || !m_filter){
      return;
    }

    if (isDynamicMesh()){
      TileCacheMap navMap;
      MObjList addDone;
      MObjList removeDone;
      if(true){
        std::lock_guard<std::mutex> locker(m_mutex_update);
        navMap.swap(m_cacheMap);
        addDone.swap(m_addDone);
        removeDone.swap(m_removeDone);
        m_cacheMap.clear();
        m_addDone.clear();
        m_removeDone.clear();
      }

      //notify navmesh update success
      if (m_nofity){
        m_nofity(addDone, removeDone);
      }

      //update navmesh
      for (auto& pair : navMap){
        auto& cache = pair.second;
        m_navMesh->removeTile(m_navMesh->getTileRefAt(cache.index.first, cache.index.second, 0), 0, 0);
        if (cache.data){
          dtStatus status = m_navMesh->addTile(cache.data, cache.size, DT_TILE_FREE_DATA, 0, 0);
          if (dtStatusFailed(status)){
            dtFree(cache.data);
          }
        }
      }
    }
  }

  bool NavRuntime::saveDump(const std::string& path) {
    if (!isDynamicMesh()){
      return false;
    }
    std::lock_guard<std::mutex> locker(m_mutex_run);
    return NavScene::saveDump(path);
  }

  bool NavRuntime::loadDump(const std::string& path){
    if (!isDynamicMesh()){
      return false;
    }
    std::lock_guard<std::mutex> locker(m_mutex_run);
    return NavScene::loadDump(path);
  }

  void NavRuntime::modify(const std::vector<WorldItem>& items, const std::vector<MObjId>& ids){
    if (!isDynamicMesh()){
      return ;
    }
    for (auto& item : items){
      assert(m_meshs.hasData(item.m_mesh));
    }
    std::lock_guard<std::mutex> locker(m_mutex_run);
    m_waitAdd.insert(m_waitAdd.end(), items.begin(), items.end());
    m_waitRemove.insert(m_waitRemove.end(), ids.begin(), ids.end());
  }

  bool NavRuntime::addMesh(const MeshPtr& mesh){
    if (!isDynamicMesh()){
      return true;
    }
    assert(!m_threadRun);
    if (!mesh){ return false; }
    return m_meshs.addData(mesh->id(), mesh);
  }

  bool NavRuntime::buildTile(const TileIndex& tile){
    if (!isDynamicMesh()){
      assert(false);
      return false;
    }
    rcConfig cf;
    rcHeightfield* solid = rcAllocHeightfield();
    rcCompactHeightfield* chf = rcAllocCompactHeightfield();
    rcPolyMesh* pmesh = rcAllocPolyMesh();
    rcPolyMeshDetail* dmesh = rcAllocPolyMeshDetail();
    rcContourSet* cset = rcAllocContourSet();
    TileNavCache cache = buildTileCache(m_ctx, &cf, solid, chf, pmesh, dmesh, cset, tile);
    rcFreeHeightField(solid);
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);

    if (cache.data == nullptr || cache.size == 0){
      return false;
    }
    std::lock_guard<std::mutex> locker(m_mutex_update);
    m_cacheMap[cache.index] = cache;
    return true;
  }

  


  void NavRuntime::threadMain(){
    MObjList removes;
    VItems adds;
    std::set<TileIndex> tiles;

    while (m_threadRun){
      tiles.clear();
      adds.clear();
      removes.clear();

      if(true){
        std::lock_guard<std::mutex> locker(m_mutex_run);
        adds = m_waitAdd;
        removes = m_waitRemove;
        m_waitAdd.clear();
        m_waitRemove.clear();
      }

      m_ctx->startTimer(RC_TIMER_TOTAL);
      MObjList ids;
      for (auto& item : adds){
        ids.push_back(item.m_id);
      }

      
      auto it = std::remove_if(adds.begin(), adds.end(), [&removes](const WorldItem& item){
        return std::find(removes.begin(), removes.end(), item.m_id) != removes.end();
      });
      adds.erase(it, adds.end());

    
      std::vector<ObjectPtr>  rebuilds;
      for (auto& item : adds){
       
        auto mesh = m_meshs.getData(item.m_mesh);
        if (!mesh){
          assert(false);
          continue;
        }
        m_ctx->startTimer(RC_TIMER_TEMP);
        ObjectPtr ptr = std::make_shared<MeshObject>(mesh, item);
        m_ctx->stopTimer(RC_TIMER_TEMP);
        if (!ptr){
          assert(false);
          continue;
        }

        m_ctx->slog(RC_LOG_PROGRESS,
                    "add object:", mesh->name(),
                    " verts:", mesh->m_verts.count(),
                    " tris:", mesh->m_tris.count(),
                    " cost:", m_ctx->getAccumulatedTime(RC_TIMER_TEMP));

        if(m_objects.addData(ptr->id(), ptr)){
          rebuilds.push_back(ptr);
        }
        
      }

      for (auto id : removes){
        auto ptr = m_objects.eraseData(id);
        if (ptr){
          rebuilds.push_back(ptr);
        }
      }

      auto& set = setting();
      const float tcs = set.tileSize * set.cellSize;
      const float* bmin = set.navBmin;
      const float* bmax = set.navBmax;
      float delMin[3], delMax[3];
      for (auto& ptr : rebuilds){
        rcVsub(delMin, ptr->m_bouns.bmin.data(), bmin);
        rcVsub(delMax, ptr->m_bouns.bmax.data(), bmin);
        int xMin = std::floor(delMin[0] / tcs);
        int yMin = std::floor(delMin[2] / tcs);
        int xMax = std::ceil(delMax[0] / tcs);
        int yMax = std::ceil(delMax[2] / tcs);

        for (int x = xMin; x < xMax; x++){
          for (int y = yMin; y < yMax; y++){
            tiles.insert(std::make_pair(x, y));
          }
        }
      }

      for (auto& tile : tiles){
        buildTile(tile);
      }
      m_ctx->stopTimer(RC_TIMER_TOTAL);
      if (ids.size() || removes.size()){
        std::lock_guard<std::mutex> locker(m_mutex_update);
        m_addDone.insert(m_addDone.end(), ids.begin(), ids.end());
        m_removeDone.insert(m_removeDone.end(), removes.begin(), removes.end());
        m_ctx->slog(RC_LOG_PROGRESS, "build cost:", m_ctx->getAccumulatedTime(RC_TIMER_TOTAL), " add:", ids.size(), " remove:", removes.size());
      }
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    std::cout << "[navmesh] exit thread " << std::this_thread::get_id() << std::endl;
  }







}