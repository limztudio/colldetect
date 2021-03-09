#include "pch.h"

#include <vector>

#define _XM_SSE4_INTRINSICS_
#include <DirectXMath/Inc/DirectXMath.h>
#include <DirectXMath/Inc/DirectXCollision.h>

#include <combaseapi.h>
#include <tbb/tbb.h>
#include <tbb/task.h>

namespace DX = DirectX;


#pragma pack(push, 1)
union DepthMap{
    struct{
        unsigned char _00, _01, _02, _03, _04;
        unsigned char _05, _06, _07, _08;
        unsigned char _09, _10, _11;
        unsigned char _12, _13;
        unsigned char _14;
    };
    unsigned char raw[15];
};
union DepthMapList{
    struct{
        DepthMap _0, _1, _2, _3;
    };
    DepthMap raw[4];
};
#pragma pack(pop)

union alignas(16) Triangle{
    struct alignas(16){
        DX::XMFLOAT3A _0, _1, _2;
    };
    DX::XMFLOAT3A raw[3];
};
struct Collider{
    DX::BoundingBox aabb;
    std::vector<Triangle, aligned_allocator<Triangle, 16>> triangles;
};


static const size_t glb_depthIndexTable[] = {
    0,
    5,  1,
    9,  6,  2,
    12, 10, 7,  3,
    14, 13, 11, 8,  4
};

static tbb::task_group glb_worker;

static std::vector<Collider> glb_colliders;
static std::vector<DepthMapList> glb_depthMapList;
static std::vector<unsigned char> glb_occludeMap;


static float __vectorcall lcl_intersect(DX::XMVECTOR xmm_origin, DX::XMVECTOR xmm_normal){
    float fDist = FLT_MAX;
    float fOut;
    
    bool bIntersected = false;

    DX::BoundingSphere sphere;
    DX::XMStoreFloat3(&sphere.Center, xmm_origin);
    sphere.Radius = 0.0001f;

    for(const auto& iCol : glb_colliders){
        if(iCol.aabb.Intersects(xmm_origin, xmm_normal, fOut)){
            for(const auto& iTri : iCol.triangles){
                auto xmm_v0 = DX::XMLoadFloat3A(&iTri._0);
                auto xmm_v1 = DX::XMLoadFloat3A(&iTri._1);
                auto xmm_v2 = DX::XMLoadFloat3A(&iTri._2);

                if(sphere.Intersects(xmm_v0, xmm_v1, xmm_v2))
                    return 0.f;

                if(DX::TriangleTests::Intersects(xmm_origin, xmm_normal, xmm_v0, xmm_v1, xmm_v2, fOut)){
                    fDist = std::min(fDist, fOut);
                    bIntersected = true;
                }
            }
        }
    }

    return bIntersected ? fDist : -FLT_MAX;
}

static void lcl_fillDepthFaceInfo(DX::XMFLOAT3 flt3_origin, DX::XMFLOAT3 flt3_normal, float fMaxDist, unsigned char* pDepth){
    static const DX::XMVECTORF32 xmmvar_pass = { { { -9000.f, -9000.f, -9000.f, -9000.f } } };
    static const DX::XMVECTORF32 xmmvar_small = { { { -0.0001f, -0.0001f, -0.0001f, -0.0001f } } };
    static const DX::XMVECTORF32 xmmvar_255 = { { { 255.f, 255.f, 255.f, 255.f } } };

    auto xmm_origin = DX::XMLoadFloat3(&flt3_origin);
    auto xmm_normal = DX::XMLoadFloat3(&flt3_normal);

    auto fOut = lcl_intersect(xmm_origin, xmm_normal);
    auto xmms_out = DX::XMVectorReplicate(fOut);
    auto xmms_maxDist = DX::XMVectorReplicate(fMaxDist);

    if((_mm_movemask_ps(DX::XMVectorLess(xmms_out, xmmvar_pass)) & 0x01) == 1)
        (*pDepth) = (unsigned char)(0xff);
    else if((_mm_movemask_ps(DX::XMVectorLess(xmms_out, xmmvar_small)) & 0x01) == 1)
        (*pDepth) = (unsigned char)(0x00);
    else if((_mm_movemask_ps(DX::XMVectorGreaterOrEqual(DX::XMVectorSubtract(xmms_out, xmms_maxDist), xmmvar_small)) & 0x01) == 1)
        (*pDepth) = (unsigned char)(0xff);
    else{
        auto xmms_out = DX::XMVectorReplicate(fOut);
        xmms_out = DX::XMVectorDivide(xmms_out, xmms_maxDist);
        xmms_out = DX::XMVectorMultiply(xmms_out, xmmvar_255);

        auto iOut = (int)(DX::XMVectorGetX(xmms_out));
        if(iOut < 0)
            iOut = 0;
        if(iOut > 255)
            iOut = 255;

        (*pDepth) = (unsigned char)(iOut);
    }
}
static void __vectorcall lcl_fillDepthTriInfo(DX::XMVECTOR xmm_origin, DX::XMVECTOR xmm_v0, DX::XMVECTOR xmm_v1, DX::XMVECTOR xmm_v2, DepthMap* pDepthMap){
    static const DX::XMVECTORF32 xmmvar_small = { { { 0.0001f, 0.0001f, 0.0001f, 0.0001f } } };

    // (0,0)      (1,0)
    //  v0 ------- v1
    //  |         /
    //  |        /
    //  |       /
    //  |      /
    //  |     /
    //  |    /
    //  |   /
    //  |  /
    //  | /
    //  v2
    // (0,1)

    DX::XMFLOAT3 flt3_origin;
    DX::XMStoreFloat3(&flt3_origin, xmm_origin);

    unsigned iID = 0;

    for(unsigned i = 0; i <= 4; ++i){
        auto xmm_i = DX::XMVectorReplicate(i * 0.25f);
        auto xmm_p = DX::XMVectorLerpV(xmm_v0, xmm_v2, xmm_i);
        auto xmm_q = DX::XMVectorLerpV(xmm_v0, xmm_v1, xmm_i);

        for(unsigned j = 0; j <= i; ++j){
            auto xmm_j = DX::XMVectorReplicate(j * 0.25f);
            auto xmm_target = DX::XMVectorLerpV(xmm_p, xmm_q, xmm_j);
            auto xmm_diff = DX::XMVectorSubtract(xmm_target, xmm_origin);
            auto xmms_lenSq = DX::XMVector3LengthSq(xmm_diff);

            auto* pDepth = &pDepthMap->raw[glb_depthIndexTable[iID++]];
            if((_mm_movemask_ps(DX::XMVectorLess(xmms_lenSq, xmmvar_small)) & 0x01) == 1)
                (*pDepth) = 0x00;
            else{
                auto xmms_len = DX::XMVectorSqrt(xmms_lenSq);
                auto xmm_normal = DX::XMVectorDivide(xmm_diff, xmms_len);

                auto fLen = DX::XMVectorGetX(xmms_len);

                DX::XMFLOAT3 flt3_normal;
                DX::XMStoreFloat3(&flt3_normal, xmm_normal);

                glb_worker.run([=](){ lcl_fillDepthFaceInfo(flt3_origin, flt3_normal, fLen, pDepth); });
            }
        }
    }
}


extern "C" __declspec(dllexport) void _cdecl CDReserveColliderTable(unsigned long numColl){
    glb_worker.wait();

    glb_colliders.clear();
    glb_colliders.reserve(numColl);
}
extern "C" __declspec(dllexport) bool _cdecl CDAddCollider(float* vertices, unsigned long numVert){
    if(numVert % 9)
        return false;

    glb_worker.wait();

    auto xmm_min = DX::XMVectorReplicate(FLT_MAX);
    auto xmm_max = DX::XMVectorReplicate(-FLT_MAX);

    Collider newCol;
    newCol.triangles.reserve(numVert / 9);
    for(auto i = decltype(numVert){ 0 }; i < numVert; i += 9){
        Triangle newTri;
        {
            auto xmm_p = DX::XMVectorSet(vertices[i + 0], vertices[i + 1], vertices[i + 2], 0.f);

            xmm_min = DX::XMVectorMin(xmm_min, xmm_p);
            xmm_max = DX::XMVectorMax(xmm_max, xmm_p);

            DX::XMStoreFloat3A(&newTri._0, xmm_p);
        }
        {
            auto xmm_p = DX::XMVectorSet(vertices[i + 3], vertices[i + 4], vertices[i + 5], 0.f);

            xmm_min = DX::XMVectorMin(xmm_min, xmm_p);
            xmm_max = DX::XMVectorMax(xmm_max, xmm_p);

            DX::XMStoreFloat3A(&newTri._1, xmm_p);
        }
        {
            auto xmm_p = DX::XMVectorSet(vertices[i + 6], vertices[i + 7], vertices[i + 8], 0.f);

            xmm_min = DX::XMVectorMin(xmm_min, xmm_p);
            xmm_max = DX::XMVectorMax(xmm_max, xmm_p);

            DX::XMStoreFloat3A(&newTri._2, xmm_p);
        }
        newCol.triangles.emplace_back(std::move(newTri));
    }
    DX::BoundingBox::CreateFromPoints(newCol.aabb, xmm_min, xmm_max);

    glb_colliders.emplace_back(std::move(newCol));

    return true;
}

extern "C" __declspec(dllexport) void _cdecl CDFillDepthInfo(const float* rawVertices, unsigned char* rawDepthMap, unsigned char* rawOccludeMap, unsigned long numTet){
    glb_worker.wait();

    {
        glb_depthMapList.clear();
        glb_depthMapList.resize(numTet);

        for(auto iTet = decltype(numTet){ 0 }; iTet < numTet; ++iTet){
            auto xmm_v0 = DX::XMVectorSet(rawVertices[(iTet * 4 * 3) + (0 * 3) + 0], rawVertices[(iTet * 4 * 3) + (0 * 3) + 1], rawVertices[(iTet * 4 * 3) + (0 * 3) + 2], 0.f);
            auto xmm_v1 = DX::XMVectorSet(rawVertices[(iTet * 4 * 3) + (1 * 3) + 0], rawVertices[(iTet * 4 * 3) + (1 * 3) + 1], rawVertices[(iTet * 4 * 3) + (1 * 3) + 2], 0.f);
            auto xmm_v2 = DX::XMVectorSet(rawVertices[(iTet * 4 * 3) + (2 * 3) + 0], rawVertices[(iTet * 4 * 3) + (2 * 3) + 1], rawVertices[(iTet * 4 * 3) + (2 * 3) + 2], 0.f);
            auto xmm_v3 = DX::XMVectorSet(rawVertices[(iTet * 4 * 3) + (3 * 3) + 0], rawVertices[(iTet * 4 * 3) + (3 * 3) + 1], rawVertices[(iTet * 4 * 3) + (3 * 3) + 2], 0.f);

            auto* pDepthMap0 = &glb_depthMapList[iTet]._0;
            auto* pDepthMap1 = &glb_depthMapList[iTet]._1;
            auto* pDepthMap2 = &glb_depthMapList[iTet]._2;
            auto* pDepthMap3 = &glb_depthMapList[iTet]._3;

            { // 0 -> 1, 2, 3
                lcl_fillDepthTriInfo(xmm_v0, xmm_v1, xmm_v2, xmm_v3, pDepthMap0);
            }
            { // 1 -> 0, 2, 3
                lcl_fillDepthTriInfo(xmm_v1, xmm_v0, xmm_v2, xmm_v3, pDepthMap1);
            }
            { // 2 -> 0, 1, 3
                lcl_fillDepthTriInfo(xmm_v2, xmm_v0, xmm_v1, xmm_v3, pDepthMap2);
            }
            { // 3 -> 0, 1, 2
                lcl_fillDepthTriInfo(xmm_v3, xmm_v0, xmm_v1, xmm_v2, pDepthMap3);
            }
        }
    }

    glb_worker.wait();
    CopyMemory(rawDepthMap, glb_depthMapList.data(), glb_depthMapList.size() * sizeof(DepthMapList));

    { // assume that at least two of faces are completely occluded, this tetrahedron have to be subdivided.
        glb_occludeMap.clear();
        glb_occludeMap.resize(numTet, 0x00);

        tbb::parallel_for(size_t(0u), size_t(numTet), [](size_t iTet){
            unsigned char cMax;
            unsigned char cFlag = 0x00;
            size_t uCount = 0;

            {
                cMax = glb_depthMapList[iTet]._0._00;
                for(size_t i = 1; i < _countof(DepthMap::raw) - 1u; ++i)
                    cMax = std::max(cMax, glb_depthMapList[iTet]._0.raw[i]);
            }
            if(cMax < 0xff){
                cFlag |= (unsigned char)(0x01u << 0u);
                ++uCount;
            }

            {
                cMax = glb_depthMapList[iTet]._1._00;
                for(size_t i = 1; i < _countof(DepthMap::raw) - 1u; ++i)
                    cMax = std::max(cMax, glb_depthMapList[iTet]._1.raw[i]);
            }
            if(cMax < 0xff){
                cFlag |= (unsigned char)(0x01u << 1u);
                ++uCount;
            }

            {
                cMax = glb_depthMapList[iTet]._2._00;
                for(size_t i = 1; i < _countof(DepthMap::raw) - 1u; ++i)
                    cMax = std::max(cMax, glb_depthMapList[iTet]._2.raw[i]);
            }
            if(cMax < 0xff){
                cFlag |= (unsigned char)(0x01u << 2u);
                ++uCount;
            }

            {
                cMax = glb_depthMapList[iTet]._3._00;
                for(size_t i = 1; i < _countof(DepthMap::raw) - 1u; ++i)
                    cMax = std::max(cMax, glb_depthMapList[iTet]._3.raw[i]);
            }
            if(cMax < 0xff){
                cFlag |= (unsigned char)(0x01u << 3u);
                ++uCount;
            }

            glb_occludeMap[iTet] = (uCount < 2) ? 0x00 : cFlag;
            });
    }

    CopyMemory(rawOccludeMap, glb_occludeMap.data(), glb_occludeMap.size() * sizeof(unsigned char));
}


BOOL APIENTRY DllMain(
    HMODULE hModule,
    DWORD ul_reason_for_call,
    LPVOID lpReserved
)
{
    switch(ul_reason_for_call){
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}

