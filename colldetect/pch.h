#ifndef PCH_H
#define PCH_H


#define NOMINMAX


#include <new>

#include <tbb/tbbmalloc_proxy.h>
#include <tbb/scalable_allocator.h>

#include "framework.h"


template<typename T, size_t AlignedSize>
class aligned_allocator{
public:
    typedef T                        value_type;
    typedef T*                       pointer;
    typedef const T*                 const_pointer;
    typedef T&                       reference;
    typedef const T&                 const_reference;
    typedef decltype(AlignedSize)    size_type;
    typedef ptrdiff_t                difference_type;

    typedef std::true_type propagate_on_container_move_assignment;

    template<typename U>
    struct rebind{ typedef aligned_allocator<U, AlignedSize> other; };


public:
    inline aligned_allocator()noexcept{}
    template<typename U>
    inline aligned_allocator(const aligned_allocator<U, AlignedSize>&)noexcept{}


public:
    inline pointer allocate(size_type n, typename aligned_allocator<void, AlignedSize>::const_pointer = 0){
        void* ptr = scalable_aligned_malloc(n * sizeof(T), AlignedSize);
        if(!ptr)
            throw std::bad_alloc();

        return reinterpret_cast<pointer>(ptr);
    }
    inline void deallocate(pointer p, size_type)noexcept{
        return scalable_aligned_free(p);
    }

    template<typename U, typename... ARGS>
    inline void construct(U* p, ARGS&&... args){
        ::new(reinterpret_cast<void*>(p)) U(std::forward<ARGS>(args)...);
    }
    inline void destroy(pointer p){
        p->~T();
    }

public:
    inline size_type max_size()const noexcept{
        return (size_type(~0) - size_type(AlignedSize)) / sizeof(T);
    }
    inline pointer address(reference x)const noexcept{
        return std::addressof(x);
    }
    inline const_pointer address(const_reference x)const noexcept{
        return std::addressof(x);
    }
};


template<size_t AlignedSize>
class aligned_allocator<void, AlignedSize>{
public:
    typedef void*             pointer;
    typedef const void*       const_pointer;
    typedef void              value_type;

    template<typename U>
    struct rebind{ typedef aligned_allocator<U, AlignedSize> other; };
};

template<typename T, size_t AlignedSize>
class aligned_allocator<const T, AlignedSize>{
public:
    typedef T                        value_type;
    typedef const T*                 pointer;
    typedef const T*                 const_pointer;
    typedef const T&                 reference;
    typedef const T&                 const_reference;
    typedef decltype(AlignedSize)    size_type;
    typedef ptrdiff_t                difference_type;

    typedef std::true_type propagate_on_container_move_assignment;

    template <typename U>
    struct rebind{ typedef aligned_allocator<U, AlignedSize> other; };


public:
    inline aligned_allocator()noexcept{}
    template<typename U>
    inline aligned_allocator(const aligned_allocator<U, AlignedSize>&)noexcept{}


public:
    inline pointer allocate(size_type n, typename aligned_allocator<void, AlignedSize>::const_pointer = 0){
        void* ptr = scalable_aligned_malloc(n * sizeof(T), AlignedSize);
        if(!ptr)
            throw std::bad_alloc();

        return reinterpret_cast<pointer>(ptr);
    }
    inline void deallocate(pointer p, size_type)noexcept{
        return scalable_aligned_free(p);
    }

    template<typename U, typename... ARGS>
    inline void construct(U* p, ARGS&&... args){
        ::new(reinterpret_cast<void*>(p)) U(std::forward<ARGS>(args)...);
    }
    inline void destroy(pointer p){
        p->~T();
    }

public:
    inline size_type max_size()const noexcept{
        return (size_type(~0) - size_type(AlignedSize)) / sizeof(T);
    }
    inline const_pointer address(const_reference x)const noexcept{
        return std::addressof(x);
    }
};


extern void* operator new(std::size_t count);
extern void* operator new[](std::size_t count);
extern void* operator new(std::size_t count, const std::nothrow_t&)noexcept;
extern void* operator new[](std::size_t count, const std::nothrow_t&)noexcept;

#ifdef __cpp_aligned_new
extern void* operator new(std::size_t count, std::align_val_t al);
extern void* operator new[](std::size_t count, std::align_val_t al);
extern void* operator new(std::size_t count, std::align_val_t al, const std::nothrow_t&)noexcept;
extern void* operator new[](std::size_t count, std::align_val_t al, const std::nothrow_t&)noexcept;
#endif


extern void operator delete(void* ptr)noexcept;
extern void operator delete[](void* ptr)noexcept;
extern void operator delete(void* ptr, std::size_t sz)noexcept;
extern void operator delete[](void* ptr, std::size_t sz)noexcept;
extern void operator delete(void* ptr, const std::nothrow_t&)noexcept;
extern void operator delete[](void* ptr, const std::nothrow_t&)noexcept;

#ifdef __cpp_aligned_new
extern void operator delete(void* ptr, std::align_val_t al)noexcept;
extern void operator delete[](void* ptr, std::align_val_t al)noexcept;
extern void operator delete(void* ptr, std::size_t sz, std::align_val_t al)noexcept;
extern void operator delete[](void* ptr, std::size_t sz, std::align_val_t al)noexcept;
extern void operator delete(void* ptr, std::align_val_t al, const std::nothrow_t&)noexcept;
extern void operator delete[](void* ptr, std::align_val_t al, const std::nothrow_t&)noexcept;
#endif


#endif //PCH_H

