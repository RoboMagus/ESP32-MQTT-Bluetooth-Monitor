#ifndef CALLBACK_HELPER_H
#define CALLBACK_HELPER_H

// Callback helper class / definitions based on the bounce implementation by embeddedartistry:
// https://github.com/embeddedartistry/embedded-resources/blob/master/examples/cpp/bounce.cpp
//
// Better lead:
//https://stackoverflow.com/questions/1000663/using-a-c-class-member-function-as-a-c-callback-function

#include <functional>


template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
   template <typename... Args> 
   static Ret callback(Args... args) {                    
      return func(args...);  
   }
   static std::function<Ret(Params...)> func; 
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;



#endif // CALLBACK_HELPER_H
