#include <vector>

#include <windows.h>


typedef bool(_cdecl*_TGBuildTets)(float* vertices, unsigned long numVert);

typedef unsigned long(_cdecl*_TGGetTetCount)(void);

typedef void(_cdecl*_TGGetTetVertices)(float* vOut);
typedef void(_cdecl*_TGGetTetIntraIndices)(unsigned long* vOut);
typedef void(_cdecl*_TGGetTetAjacentIndices)(unsigned long* vOut);
typedef void(_cdecl*_TGGetTetBaryMatrices)(float* vOut);


struct uint4{
    unsigned long raw[4];
};


int main(){
#if defined(_M_X64) || defined(__amd64__)
    HMODULE pLibrary = LoadLibrary("tetgen_x64.dll");
#else
    HMODULE pLibrary = LoadLibrary("tetgen_Win32.dll");
#endif

    _TGBuildTets TGBuildTets = reinterpret_cast<_TGBuildTets>(GetProcAddress(pLibrary, "TGBuildTets"));
    _TGGetTetCount TGGetTetCount = reinterpret_cast<_TGGetTetCount>(GetProcAddress(pLibrary, "TGGetTetCount"));
    _TGGetTetVertices TGGetTetVertices = reinterpret_cast<_TGGetTetVertices>(GetProcAddress(pLibrary, "TGGetTetVertices"));
    _TGGetTetIntraIndices TGGetTetIntraIndices = reinterpret_cast<_TGGetTetIntraIndices>(GetProcAddress(pLibrary, "TGGetTetIntraIndices"));
    _TGGetTetAjacentIndices TGGetTetAjacentIndices = reinterpret_cast<_TGGetTetAjacentIndices>(GetProcAddress(pLibrary, "TGGetTetAjacentIndices"));
    _TGGetTetBaryMatrices TGGetTetBaryMatrices = reinterpret_cast<_TGGetTetBaryMatrices>(GetProcAddress(pLibrary, "TGGetTetBaryMatrices"));

    //float vertices[] = {
    //    0, 0, 0,
    //    1, 0, 0,
    //    0, 0, 1,
    //    1, 0, 1,

    //    0, 1, 0,
    //    1, 1, 0,
    //    0, 1, 1,
    //    1, 1, 1,
    //};
    //float vertices[] = {-80, 20, -80, -40, 20, -80, 0, 20, -80, 40, 20, -80, 80, 20, -80, -80, 60, -80, -40, 60, -80, 0, 60, -80, 40, 60, -80, 80, 60, -80, -80, 100, -80, -40, 100, -80, 0, 100, -80, 40, 100, -80, 80, 100, -80, -80, 140, -80, -40, 140, -80, 0, 140, -80, 40, 140, -80, 80, 140, -80, -80, 180, -80, -40, 180, -80, 0, 180, -80, 40, 180, -80, 80, 180, -80, -80, 20, -40, -40, 20, -40, 0, 20, -40, 40, 20, -40, 80, 20, -40, -80, 60, -40, -40, 60, -40, 0, 60, -40, 40, 60, -40, 80, 60, -40, -80, 100, -40, -40, 100, -40, 0, 100, -40, 40, 100, -40, 80, 100, -40, -80, 140, -40, -40, 140, -40, 0, 140, -40, 40, 140, -40, 80, 140, -40, -80, 180, -40, -40, 180, -40, 0, 180, -40, 40, 180, -40, 80, 180, -40, -80, 20, 0, -40, 20, 0, 0, 20, 0, 40, 20, 0, 80, 20, 0, -80, 60, 0, -40, 60, 0, 0, 60, 0, 40, 60, 0, 80, 60, 0, -80, 100, 0, -40, 100, 0, 0, 100, 0, 40, 100, 0, 80, 100, 0, -80, 140, 0, -40, 140, 0, 0, 140, 0, 40, 140, 0, 80, 140, 0, -80, 180, 0, -40, 180, 0, 0, 180, 0, 40, 180, 0, 80, 180, 0, -80, 20, 40, -40, 20, 40, 0, 20, 40, 40, 20, 40, 80, 20, 40, -80, 60, 40, -40, 60, 40, 0, 60, 40, 40, 60, 40, 80, 60, 40, -80, 100, 40, -40, 100, 40, 0, 100, 40, 40, 100, 40, 80, 100, 40, -80, 140, 40, -40, 140, 40, 0, 140, 40, 40, 140, 40, 80, 140, 40, -80, 180, 40, -40, 180, 40, 0, 180, 40, 40, 180, 40, 80, 180, 40, -80, 20, 80, -40, 20, 80, 0, 20, 80, 40, 20, 80, 80, 20, 80, -80, 60, 80, -40, 60, 80, 0, 60, 80, 40, 60, 80, 80, 60, 80, -80, 100, 80, -40, 100, 80, 0, 100, 80, 40, 100, 80, 80, 100, 80, -80, 140, 80, -40, 140, 80, 0, 140, 80, 40, 140, 80, 80, 140, 80, -80, 180, 80, -40, 180, 80, 0, 180, 80, 40, 180, 80, 80, 180, 80, };
    float vertices[] = { -26.09519, -1.530641, -26.1999, 26.09519, -1.530641, -26.1999, -26.09519, 13.33064, -26.1999, -26.09519, -1.530641, 26.1999, -26.09519, 13.33064, 26.1999, 26.09519, -1.530641, 26.1999, 26.09519, 13.33064, -26.1999, 26.09519, 13.33064, 26.1999 };
    TGBuildTets(vertices, _countof(vertices));

    std::vector<uint4> indices(TGGetTetCount() << 2);
    TGGetTetAjacentIndices((unsigned long*)indices.data());

    FreeLibrary(pLibrary);
    return 0;
}