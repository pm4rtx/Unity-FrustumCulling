BUILD_DIR = .Build
SOURCE_DIR = Sources
FILE_NAME = frustumcull

# a dummy .obj file with DLL entry point
$(BUILD_DIR)\$(FILE_NAME)_null.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -fwin64 -Oxv -o $*.obj -l $*.lst $?

# SSE2 version
$(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd4_sse2.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -fwin64 -Ox -o$*.obj -l $*.lst $?

# AVX version with 256-bit registers (VEX.256). The expected input is the stream of aosoa8 = {float8 minx,miny,minz,maxx,maxy,maxz } structures
$(BUILD_DIR)\$(FILE_NAME)_aosoa8_aabb_stream_simd8_avx.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -DUSE_AVX=1 -fwin64 -Ox -o $*.obj -l $*.lst $?

# AVX version with 128-bit registers (VEX.128). The expected input is the stream of aosoa4 = {float8 minx,miny,minz,maxx,maxy,maxz } structures
$(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd4_avx.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -DUSE_AVX=1 -DSIMD_SIZE=4 -fwin64 -Ox -o $*.obj -l $*.lst $?

# AVX version with 256-bit registers (VEX.256). The expected input is the stream of aosoa4 = {float8 minx,miny,minz,maxx,maxy,maxz } structures
$(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd8_avx.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -DUSE_AVX=1 -DUSE_AOSOA4_INPUT=1 -fwin64 -Ox -o $*.obj -l $*.lst $?

# all versions same as above but with extra test against the bounding box of the frustum which removes some false positives
$(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd4_sse2.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -DUSE_EXTRATESTS=1 -fwin64 -Ox -o $*.obj -l $*.lst $?

$(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa8_aabb_stream_simd8_avx.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -fwin64 -Ox -o $*.obj -l $*.lst $?

$(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd4_avx.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -DSIMD_SIZE=4 -fwin64 -Ox -o $*.obj -l $*.lst $?

$(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd8_avx.obj: $(SOURCE_DIR)\$(FILE_NAME).asm
    nasm -DFUNCTION_NAME=$(*B) -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -DUSE_AOSOA4_INPUT=1 -fwin64 -Ox -o $*.obj -l $*.lst $?

$(BUILD_DIR)\$(FILE_NAME).dll: \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd4_sse2.obj \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa8_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd4_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd4_sse2.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa8_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd4_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_null.obj
    link /nologo /opt:ref /dll /map /LARGEADDRESSAWARE /entry:_DllMainCRTStartup /machine:x64 /verbose /pdbstripped:.$*.pdb /out:$*.dll $**

$(BUILD_DIR)\lib$(FILE_NAME).lib: \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd4_sse2.obj \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa8_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd4_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_aosoa4_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd4_sse2.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa8_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd4_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_extratests_aosoa4_aabb_stream_simd8_avx.obj \
    $(BUILD_DIR)\$(FILE_NAME)_null.obj
    lib /wx /nologo /subsystem:windows /machine:x64 /verbose /out:$*.lib $**

mkdir:
    @if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

clean:
    @if exist $(BUILD_DIR) rmdir /s /q $(BUILD_DIR)

copy:
    copy /b /y /v $(BUILD_DIR)\$(FILE_NAME).dll Assets\UnityNativePlugin-$(FILE_NAME).dll

dll: $(BUILD_DIR)\$(FILE_NAME).dll

lib: $(BUILD_DIR)\lib$(FILE_NAME).lib

build: mkdir dll lib copy

rebuild: clean build

all: rebuild
