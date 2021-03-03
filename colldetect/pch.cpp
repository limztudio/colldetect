#include "pch.h"


void* operator new(std::size_t count){
    void* ptr = scalable_malloc(count);
    if(!ptr)
        throw std::bad_alloc{};
    return ptr;
}
void* operator new[](std::size_t count){
    void* ptr = scalable_malloc(count);
    if(!ptr)
        throw std::bad_alloc{};
    return ptr;
}
void* operator new(std::size_t count, const std::nothrow_t&)noexcept{
    return scalable_malloc(count);
}
void* operator new[](std::size_t count, const std::nothrow_t&)noexcept{
    return scalable_malloc(count);
}

#ifdef __cpp_aligned_new
void* operator new(std::size_t count, std::align_val_t al){
    void* ptr = scalable_aligned_malloc(count, (size_t)al);
    if(!ptr)
        throw std::bad_alloc{};
    return ptr;
}
void* operator new[](std::size_t count, std::align_val_t al){
    void* ptr = scalable_aligned_malloc(count, (size_t)al);
    if(!ptr)
        throw std::bad_alloc{};
    return ptr;
}
void* operator new(std::size_t count, std::align_val_t al, const std::nothrow_t&)noexcept{
    return scalable_aligned_malloc(count, (size_t)al);
}
void* operator new[](std::size_t count, std::align_val_t al, const std::nothrow_t&)noexcept{
    return scalable_aligned_malloc(count, (size_t)al);
}
#endif


void operator delete(void* ptr)noexcept{
    scalable_free(ptr);
}
void operator delete[](void* ptr)noexcept{
    scalable_free(ptr);
}
void operator delete(void* ptr, std::size_t sz)noexcept{
    scalable_free(ptr);
}
void operator delete[](void* ptr, std::size_t sz)noexcept{
    scalable_free(ptr);
}
void operator delete(void* ptr, const std::nothrow_t&)noexcept{
    scalable_free(ptr);
}
void operator delete[](void* ptr, const std::nothrow_t&)noexcept{
    scalable_free(ptr);
}

#ifdef __cpp_aligned_new
void operator delete(void* ptr, std::align_val_t al)noexcept{
    scalable_aligned_free(ptr);
}
void operator delete[](void* ptr, std::align_val_t al)noexcept{
    scalable_aligned_free(ptr);
}
void operator delete(void* ptr, std::size_t sz, std::align_val_t al)noexcept{
    scalable_aligned_free(ptr);
}
void operator delete[](void* ptr, std::size_t sz, std::align_val_t al)noexcept{
    scalable_aligned_free(ptr);
}
void operator delete(void* ptr, std::align_val_t al, const std::nothrow_t&)noexcept{
    scalable_aligned_free(ptr);
}
void operator delete[](void* ptr, std::align_val_t al, const std::nothrow_t&)noexcept{
    scalable_aligned_free(ptr);
}
#endif

