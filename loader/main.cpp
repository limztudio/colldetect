#include <vector>

#include <windows.h>


typedef void(_cdecl*_CDReserveColliderTable)(unsigned long numColl);
typedef bool(_cdecl*_CDAddCollider)(float* vertices, unsigned long numVert);

typedef void(_cdecl*_CDFillDepthInfo)(const float* rawVertices, unsigned char* rawDepthMap, unsigned long numTet);


union float3{
    struct{
        float x, y, z;
    };
    float raw[3];
};
union float33{
    struct{
        float3 _0, _1, _2;
    };
    float3 raw[3];
};
union float43{
    struct{
        float3 _0, _1, _2, _3;
    };
    float3 raw[4];
};

#pragma pack(push, 1)
union byte15{
    struct{
        unsigned char _00, _01, _02, _03, _04;
        unsigned char _05, _06, _07, _08;
        unsigned char _09, _10, _11;
        unsigned char _12, _13;
        unsigned char _14;
    };
    unsigned char raw[15];
};
#pragma pack(pop)


static inline float3 _float3(float _x, float _y, float _z){
    float3 o = { _x, _y, _z };
    return o;
}
static inline float33 _float33(float3 _0, float3 _1, float3 _2){
    float33 o = { _0, _1, _2 };
    return o;
}
static inline float43 _float43(float3 _0, float3 _1, float3 _2, float3 _3){
    float43 o = { _0, _1, _2, _3 };
    return o;
}


int main(){
#if defined(_M_X64) || defined(__amd64__)
    HMODULE pLibrary = LoadLibrary("colldetect_x64.dll");
#else
    HMODULE pLibrary = LoadLibrary("colldetect_Win32.dll");
#endif

    _CDReserveColliderTable CDReserveColliderTable = reinterpret_cast<_CDReserveColliderTable>(GetProcAddress(pLibrary, "CDReserveColliderTable"));
    _CDAddCollider CDAddCollider = reinterpret_cast<_CDAddCollider>(GetProcAddress(pLibrary, "CDAddCollider"));
    _CDFillDepthInfo CDFillDepthInfo = reinterpret_cast<_CDFillDepthInfo>(GetProcAddress(pLibrary, "CDFillDepthInfo"));


    float33 cols[] = {
        _float33(
            _float3(25.f, 1.f, -22.f),
            _float3(25.f, 1.f, 22.f),
            _float3(-25.f, 1.f, -22.f)
        ),
    };

    float43 tets[] = {
        _float43(
            _float3(-26.09519f, -1.530641f, -26.1999f),
            _float3(26.09519f, -1.530641f, -26.1999f),
            _float3(-26.09519f, 13.33064f, -26.1999f),
            _float3(-26.09519f, -1.530641f, 26.1999f)
        ),
        _float43(
            _float3(-26.09519f, 13.33064f, 26.1999f),
            _float3(26.09519f, -1.530641f, 26.1999f),
            _float3(26.09519f, 13.33064f, -26.1999f),
            _float3(26.09519f, 13.33064f, 26.1999f)
        ),
    };
    byte15 depthMap[_countof(tets) << 2];

    {
        CDReserveColliderTable(1);
        CDAddCollider((float*)cols, _countof(cols) * 3 * 3);

        CDFillDepthInfo((const float*)tets, (unsigned char*)depthMap, _countof(tets));
    }

    FreeLibrary(pLibrary);
    return 0;
}