@echo off

set file_name=frustumcull

if exist .Build rmdir /s /q .Build

if not exist .Build mkdir .Build

pushd .Build

:: a dummy .obj file with DLL entry point
nasm -f win64 -Oxv ../Sources/%file_name%.asm -o %file_name%_null.obj -l %file_name%_null.lst

:: SSE2 version
set fname=%file_name%_aosoa4_aabb_stream_simd4_sse2
nasm -DFUNCTION_NAME=%fname% -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

:: AVX version with 256-bit registers (VEX.256). The expected input is the stream of aosoa8 = {float8 minx,miny,minz,maxx,maxy,maxz } structures
set fname=%file_name%_aosoa8_aabb_stream_simd8_avx
nasm -DFUNCTION_NAME=%fname% -DUSE_AVX=1 -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

:: AVX version with 128-bit registers (VEX.128). The expected input is the stream of aosoa4 = {float8 minx,miny,minz,maxx,maxy,maxz } structures
set fname=%file_name%_aosoa4_aabb_stream_simd4_avx
nasm -DFUNCTION_NAME=%fname% -DUSE_AVX=1 -DSIMD_SIZE=4 -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

:: AVX version with 256-bit registers (VEX.256). The expected input is the stream of aosoa4 = {float8 minx,miny,minz,maxx,maxy,maxz } structures
set fname=%file_name%_aosoa4_aabb_stream_simd8_avx
nasm -DFUNCTION_NAME=%fname% -DUSE_AVX=1 -DUSE_AOSOA4_INPUT=1 -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

:: all versions same as above but with extra test against the bounding box of the frustum which removes some false positives
set fname=%file_name%_extratests_aosoa4_aabb_stream_simd4_sse2
nasm -DFUNCTION_NAME=%fname% -DUSE_EXTRATESTS=1 -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

set fname=%file_name%_extratests_aosoa8_aabb_stream_simd8_avx
nasm -DFUNCTION_NAME=%fname% -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

set fname=%file_name%_extratests_aosoa4_aabb_stream_simd4_avx
nasm -DFUNCTION_NAME=%fname% -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -DSIMD_SIZE=4 -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

set fname=%file_name%_extratests_aosoa4_aabb_stream_simd8_avx
nasm -DFUNCTION_NAME=%fname% -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -DUSE_AOSOA4_INPUT=1 -fwin64 -Oxv -o %fname%.obj -l %fname%.lst ../Sources/%file_name%.asm

lib /wx /nologo /subsystem:windows /machine:x64 /verbose /out:lib%file_name%.lib^
    %file_name%_aosoa4_aabb_stream_simd4_sse2.obj^
    %file_name%_aosoa8_aabb_stream_simd8_avx.obj^
    %file_name%_aosoa4_aabb_stream_simd4_avx.obj^
    %file_name%_aosoa4_aabb_stream_simd8_avx.obj^
    %file_name%_extratests_aosoa4_aabb_stream_simd4_sse2.obj^
    %file_name%_extratests_aosoa8_aabb_stream_simd8_avx.obj^
    %file_name%_extratests_aosoa4_aabb_stream_simd4_avx.obj^
    %file_name%_extratests_aosoa4_aabb_stream_simd8_avx.obj

link /nologo /opt:ref /dll /map /LARGEADDRESSAWARE /entry:_DllMainCRTStartup /machine:X64 /verbose /pdbstripped:%file_name%.pdb /out:%file_name%.dll^
    %file_name%_aosoa4_aabb_stream_simd4_sse2.obj^
    %file_name%_aosoa8_aabb_stream_simd8_avx.obj^
    %file_name%_aosoa4_aabb_stream_simd4_avx.obj^
    %file_name%_aosoa4_aabb_stream_simd8_avx.obj^
    %file_name%_extratests_aosoa4_aabb_stream_simd4_sse2.obj^
    %file_name%_extratests_aosoa8_aabb_stream_simd8_avx.obj^
    %file_name%_extratests_aosoa4_aabb_stream_simd4_avx.obj^
    %file_name%_extratests_aosoa4_aabb_stream_simd8_avx.obj^
    %file_name%_null.obj

copy /b /y /v %file_name%.dll ..\Assets\UnityNativePlugin-%file_name%.dll

popd
