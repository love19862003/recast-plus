/********************************************************************
    Filename: GuardFunction.h
    Description:
    Version:  1.0
    Created:  30:5:2016   18:39
	
    Compiler: gcc vc
    Author:   wufan, love19862003@163.com
    Organization: lezhuogame
*********************************************************************/
#ifndef __GuardFunction_H__
#define __GuardFunction_H__
#include <functional>

  namespace Utility{

    template< typename Fun>
    struct GuardObject{
      typedef Fun FunType;
      explicit GuardObject(const FunType& fun): _fun(fun){

      }
      ~GuardObject(){
        _fun();
      }
      FunType _fun;
    };

    typedef GuardObject<std::function<void()>> GuardFun; 

  }

#endif // __GuardFunction_H__
