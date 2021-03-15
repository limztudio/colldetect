#include "pch.h"

#include <vector>
#include <string>

#define _XM_SSE4_INTRINSICS_
#include <DirectXMath/Inc/DirectXMath.h>
#include <DirectXMath/Inc/DirectXCollision.h>

#include <combaseapi.h>
#include <tbb/tbb.h>
#include <tbb/task.h>

#include "robin_hood.h"


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

struct CacheDepthKey{
    DX::XMFLOAT3 Point;
    DX::XMFLOAT3 Tri0, Tri1, Tri2;
};
struct CacheDepthKeyHasher{
    inline size_t operator()(const CacheDepthKey& v)const{
        std::string str;
        str = "X" + std::to_string(v.Point.x) + "X" + std::to_string(v.Point.y) + "X" + std::to_string(v.Point.z) + "X";
        str += "Y" + std::to_string(v.Tri0.x) + "Y" + std::to_string(v.Tri0.y) + "Y" + std::to_string(v.Tri0.z) + "Y";
        str += "Z" + std::to_string(v.Tri1.x) + "Z" + std::to_string(v.Tri1.y) + "Z" + std::to_string(v.Tri1.z) + "Z";
        str += "W" + std::to_string(v.Tri2.x) + "W" + std::to_string(v.Tri2.y) + "W" + std::to_string(v.Tri2.z) + "W";

        return robin_hood::hash_bytes(str.data(), sizeof(char) * str.size());
    }
};
static inline bool operator==(const CacheDepthKey& lhs, const CacheDepthKey& rhs){
    if(lhs.Point.x != rhs.Point.x)
        return false;
    if(lhs.Point.y != rhs.Point.y)
        return false;
    if(lhs.Point.z != rhs.Point.z)
        return false;
    if(lhs.Tri0.x != rhs.Tri0.x)
        return false;
    if(lhs.Tri0.y != rhs.Tri0.y)
        return false;
    if(lhs.Tri0.z != rhs.Tri0.z)
        return false;
    if(lhs.Tri1.x != rhs.Tri1.x)
        return false;
    if(lhs.Tri1.y != rhs.Tri1.y)
        return false;
    if(lhs.Tri1.z != rhs.Tri1.z)
        return false;
    if(lhs.Tri2.x != rhs.Tri2.x)
        return false;
    if(lhs.Tri2.y != rhs.Tri2.y)
        return false;
    if(lhs.Tri2.z != rhs.Tri2.z)
        return false;
    return true;
}


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
static robin_hood::unordered_map<CacheDepthKey, DepthMap, CacheDepthKeyHasher> glb_cachedDepthMap;


static float __vectorcall lcl_intersect(DX::XMVECTOR xmm_origin, DX::XMVECTOR xmm_normal){
    static const DX::XMVECTORF32 xmmvar_radius = { { { 0.0001f, 0.0001f, 0.0001f, 0.0001f } } };
    static const DX::XMVECTORF32 xmmvar_radiusNeg = { { { -xmmvar_radius.f[0], -xmmvar_radius.f[1], -xmmvar_radius.f[2], -xmmvar_radius.f[3] } } };
    static const DX::XMVECTORF32 xmmvar_radiusSq = { { { xmmvar_radius.f[0] * xmmvar_radius.f[0], xmmvar_radius.f[1] * xmmvar_radius.f[1], xmmvar_radius.f[2] * xmmvar_radius.f[2], xmmvar_radius.f[3] * xmmvar_radius.f[3] } } };

    float fDist = FLT_MAX;
    float fOut;
    
    bool bIntersected = false;

    for(const auto& iCol : glb_colliders){
        if(iCol.aabb.Intersects(xmm_origin, xmm_normal, fOut)){
            for(const auto& iTri : iCol.triangles){
                auto xmm_v0 = DX::XMLoadFloat3A(&iTri._0);
                auto xmm_v1 = DX::XMLoadFloat3A(&iTri._1);
                auto xmm_v2 = DX::XMLoadFloat3A(&iTri._2);

                auto xmm_cross = DX::XMVector3Normalize(DX::XMVector3Cross(DX::XMVectorSubtract(xmm_v1, xmm_v0), DX::XMVectorSubtract(xmm_v2, xmm_v0)));

                // check if there are any geometries located too close
                if(!DX::XMVector3Equal(xmm_cross, DX::XMVectorZero())){
                    // Find the nearest feature on the triangle to the sphere.
                    auto xmms_dist = DX::XMVector3Dot(DX::XMVectorSubtract(xmm_origin, xmm_v0), xmm_cross);

                    // If the center of the sphere is farther from the plane of the triangle than
                    // the radius of the sphere, then there cannot be an intersection.
                    auto xmms_nointer = DX::XMVectorLess(xmms_dist, xmmvar_radiusNeg);
                    xmms_nointer = DX::XMVectorOrInt(xmms_nointer, DX::XMVectorGreater(xmms_dist, xmmvar_radius));

                    // Project the center of the sphere onto the plane of the triangle.
                    auto xmm_p = DX::XMVectorNegativeMultiplySubtract(xmm_cross, xmms_dist, xmm_origin);

                    // Is it inside all the edges? If so we intersect because the distance
                    // to the plane is less than the radius.
                    auto xmm_inter = DX::Internal::PointOnPlaneInsideTriangle(xmm_p, xmm_v0, xmm_v1, xmm_v2);

                    // Edge 0,1
                    xmm_p = DX::Internal::PointOnLineSegmentNearestPoint(xmm_v0, xmm_v1, xmm_origin);

                    // If the distance to the center of the sphere to the point is less than
                    // the radius of the sphere then it must intersect.
                    xmm_inter = DX::XMVectorOrInt(xmm_inter, DX::XMVectorLessOrEqual(DX::XMVector3LengthSq(DX::XMVectorSubtract(xmm_origin, xmm_p)), xmmvar_radiusSq));

                    // Edge 1,2
                    xmm_p = DX::Internal::PointOnLineSegmentNearestPoint(xmm_v1, xmm_v2, xmm_origin);

                    // If the distance to the center of the sphere to the point is less than
                    // the radius of the sphere then it must intersect.
                    xmm_inter = DX::XMVectorOrInt(xmm_inter, DX::XMVectorLessOrEqual(DX::XMVector3LengthSq(DX::XMVectorSubtract(xmm_origin, xmm_p)), xmmvar_radiusSq));

                    // Edge 2,0
                    xmm_p = DX::Internal::PointOnLineSegmentNearestPoint(xmm_v2, xmm_v0, xmm_origin);

                    // If the distance to the center of the sphere to the point is less than
                    // the radius of the sphere then it must intersect.
                    xmm_inter = DX::XMVectorOrInt(xmm_inter, DX::XMVectorLessOrEqual(DX::XMVector3LengthSq(DX::XMVectorSubtract(xmm_origin, xmm_p)), xmmvar_radiusSq));

                    if(DX::XMVector4EqualInt(DX::XMVectorAndCInt(xmm_inter, xmms_nointer), DX::XMVectorTrueInt()))
                        return 0.f;
                }

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

    CacheDepthKey key;
    key.Point = flt3_origin;
    DX::XMStoreFloat3(&key.Tri0, xmm_v0);
    DX::XMStoreFloat3(&key.Tri1, xmm_v1);
    DX::XMStoreFloat3(&key.Tri2, xmm_v2);

    auto itFind = glb_cachedDepthMap.find(key);
    if(itFind != glb_cachedDepthMap.cend())
        CopyMemory(pDepthMap->raw, itFind->second.raw, sizeof(DepthMap));
    else{
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
}


extern "C" __declspec(dllexport) void _cdecl CDReserveColliderTable(unsigned long numColl){
    glb_worker.wait();

    glb_colliders.clear();
    glb_colliders.reserve(numColl);

    glb_cachedDepthMap.clear();
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

    glb_cachedDepthMap.clear();

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

    {
        glb_cachedDepthMap.rehash(numTet << 2);

        for(auto iTet = decltype(numTet){ 0 }; iTet < numTet; ++iTet){
            DX::XMFLOAT3 flt3_v0 = { rawVertices[(iTet * 4 * 3) + (0 * 3) + 0], rawVertices[(iTet * 4 * 3) + (0 * 3) + 1], rawVertices[(iTet * 4 * 3) + (0 * 3) + 2] };
            DX::XMFLOAT3 flt3_v1 = { rawVertices[(iTet * 4 * 3) + (1 * 3) + 0], rawVertices[(iTet * 4 * 3) + (1 * 3) + 1], rawVertices[(iTet * 4 * 3) + (1 * 3) + 2] };
            DX::XMFLOAT3 flt3_v2 = { rawVertices[(iTet * 4 * 3) + (2 * 3) + 0], rawVertices[(iTet * 4 * 3) + (2 * 3) + 1], rawVertices[(iTet * 4 * 3) + (2 * 3) + 2] };
            DX::XMFLOAT3 flt3_v3 = { rawVertices[(iTet * 4 * 3) + (3 * 3) + 0], rawVertices[(iTet * 4 * 3) + (3 * 3) + 1], rawVertices[(iTet * 4 * 3) + (3 * 3) + 2] };

            { // 0 -> 1, 2, 3
                CacheDepthKey key;

                key.Point = flt3_v0;
                key.Tri0 = flt3_v1;
                key.Tri1 = flt3_v2;
                key.Tri2 = flt3_v3;

                glb_cachedDepthMap.emplace(std::move(key), glb_depthMapList[iTet]._0);
            }
            { // 1 -> 0, 2, 3
                CacheDepthKey key;

                key.Point = flt3_v1;
                key.Tri0 = flt3_v0;
                key.Tri1 = flt3_v2;
                key.Tri2 = flt3_v3;

                glb_cachedDepthMap.emplace(std::move(key), glb_depthMapList[iTet]._1);
            }
            { // 2 -> 0, 1, 3
                CacheDepthKey key;

                key.Point = flt3_v2;
                key.Tri0 = flt3_v0;
                key.Tri1 = flt3_v1;
                key.Tri2 = flt3_v3;

                glb_cachedDepthMap.emplace(std::move(key), glb_depthMapList[iTet]._2);
            }
            { // 3 -> 0, 1, 2
                CacheDepthKey key;

                key.Point = flt3_v3;
                key.Tri0 = flt3_v0;
                key.Tri1 = flt3_v1;
                key.Tri2 = flt3_v2;

                glb_cachedDepthMap.emplace(std::move(key), glb_depthMapList[iTet]._3);
            }
        }
    }
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

