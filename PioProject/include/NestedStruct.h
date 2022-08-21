#ifndef NESTED_STRUCT_H
#define NESTED_STRUCT_H

template<typename T, int N>
struct NestedStruct
{  
  NestedStruct<T, N-1> contained;
  T t;
  // Constructor
  NestedStruct<T, N>() : t(N) {}
};

template<typename T>
struct NestedStruct<T, 0> 
{
  T t;
  // Constructor
  NestedStruct<T, 0>() : t(0) {}
};

template<typename T, int N>
class NestWrapper
{
public:
    T (&data)[N] = reinterpret_cast<T(&)[N]>(nest);
    static const int size = N;
private:
    NestedStruct<T, N> nest;
};    

#endif // NESTED_STRUCT_H