/************************************************
 * \file NavPool.h
 * \date 2019/01/10 17:26
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
#include <functional>
#include <assert.h>
#include <type_traits>
#include <memory>
#include "NavCommon.h"
#include <string.h>
namespace NavSpace{

  template<typename T, size_t _Size = 1>
  class Pool : public NonCopyAble{
  public:
    typedef T VALUE;
    typedef std::array<T, _Size> ARRAY;

    
    static constexpr size_t ObjectSize() { return  sizeof(VALUE) * _Size; }

    explicit Pool(size_t max = 0) : m_max(max), m_count(0), m_pool(nullptr){
      static_assert(std::is_pod<T>::value);
      if (m_max > 0){
        m_pool = (VALUE*)malloc(m_max * ObjectSize());
        memset(m_pool, 0, m_max * ObjectSize());
      }
    }

    virtual ~Pool(){
      m_max = m_count = 0;
      if (m_pool) free(m_pool);
      m_pool = nullptr;
    }


    void resize(size_t size){
      reallocPool(size);
    }

    void reset(){
      m_count = 0;
      memset(m_pool, 0, m_max * ObjectSize());
    }

    bool add(const VALUE* p, size_t count = 1){
      if (m_count + count > m_max){
        size_t s = 2 * m_max;
        s = std::max<size_t>(s, 8);
        s = std::max<size_t>(s, m_count + count);
        reallocPool(s);
      }
      memcpy(&m_pool[m_count * _Size], p, count * ObjectSize());
      m_count += count;
      return true;
    }

    bool add(const Pool& pool){
      return add(pool.pool(), pool.count());
    }


    bool remove(size_t start, size_t count = 1){
      if (start  + count > m_count){
        return false;
      }

      size_t left = m_count - start - count;
      const VALUE* src = &m_pool[(start + count) * _Size];
      VALUE* dest = &m_pool[start * _Size];
      memmove(dest, src, left * ObjectSize());
      m_count -= count;
      return true;
    }

    size_t remove_if(const std::function<bool(const VALUE*, size_t)>& fun){
      size_t index = 0;
      size_t count = 0;

      while (index < m_count){
        if(fun(&m_pool[index * _Size], index) && remove(index)){
          ++count;
        }else{
          ++index;
        }
      }
      return count;
    }

    void call(const std::function<void(VALUE*, size_t)>& fun, size_t start = 0, size_t count = 0){
       if (start >= m_count){
         return;
       }

       size_t index = start;
       size_t end = m_count;
       if (count > 0){ end = std::min<size_t>(start + count, end); }
       while (index < end){
         fun(&m_pool[index * _Size], index);
         ++index;
       }
       
    }

    void call(const std::function<void(const VALUE*, size_t)>& fun, size_t start = 0, size_t count = 0) const{
      if (start >= m_count){
        return;
      }
      size_t index = start;
      size_t end = m_count;  
      if(count > 0){ end = std::min<size_t>(start + count, end); }

      while (index < end){
        fun(&m_pool[index * _Size], index);
        ++index;
      }
    }

    inline size_t count() const{ return m_count; }

    inline size_t maxCount() const{ return m_max; }

    inline const VALUE* pool(size_t index = 0) const{ /*assert(index < m_count);*/ return m_pool + index * _Size; }

    inline const VALUE& value(size_t index = 0) const{ assert(index < m_count);  return *pool(index); }


  private:
    void reallocPool(size_t size){
      if (size <= m_max) return;
      if (nullptr == m_pool){
        m_pool = (VALUE*)malloc(size * ObjectSize());
        memset(m_pool, 0, size * ObjectSize());
      }else{
        m_pool = (VALUE*)realloc(m_pool, size * ObjectSize());
      }
      m_max = size;
    }

  private:
    size_t m_max;
    size_t m_count;
    VALUE* m_pool;
  };
}