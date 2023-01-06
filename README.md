# Unity-FrustumCulling

The main motivation behind [the initial version of the sample](https://github.com/reinsteam/Unity-FrustumCulling) was
exploring how [the Burst compiler](https://docs.unity3d.com/Packages/com.unity.burst@1.8/manual/index.html) can work
together with [the Mathematics package](https://docs.unity3d.com/Packages/com.unity.mathematics@1.2/manual/index.html)
to improve the performance of a computationally intensive task where SIMD'fication and auto-vectorisation are desirable.

It was also interesting to compare the performance of the code produced by Burst against native code written in assembly,
compiled with [the Netwide Assembler (NASM)](https://nasm.us/) and imported as a native plugin.

The decision on choosing culling multiple axis-aligned bounding boxes against a frustum as an example task was made because
of two reasons:
- It is a relatively simple routine which is present in every (?) game engine and therefore seemed quite practical
- Some game engines tend to overcomplicate this routine by introducing various acceleration structures even despite
  several in-depth articles written more than a decade ago, for example, this [series of blog posts](https://zeux.io/2009/01/31/view-frustum-culling-optimization-introduction/)
  by [Arseny Kapoulkine](https://github.com/zeux/) and this [blog post](https://fgiesen.wordpress.com/2010/10/17/view-frustum-culling/)
  by [Fabian Giesen](https://github.com/rygorous), show that brute-force, SIMD'fied approaches work just fine.

## Performance

The next table summarises time required to cull 100 000 axis-aligned bounding boxes in a test scene on i7-4700 HQ while
using various approaches

|                                                                                               | Time (i7-4700 HQ)
|----------------------------------------------------------------------------------------------:|:---------------
| Standard C# code                                                                              | ~115 ms
| **Unity.Mathematics**                                                                         | ~150 ms
| **Unity.Mathematics** + Multithreading (via **Unity.Jobs**)                                   | ~41.0 - 46.0 ms
| **Unity.Mathematics** + **Burst**                                                             | ~0.76 - 0.93 ms
| **Unity.Mathematics** + Multithreading (via **Unity.Jobs**) + **Burst**                       | ~0.21 - 0.36 ms
| x86-64 native code (SSE2) via [**Native Plugin**](https://docs.unity3d.com/Manual/NativePlugins.html) (single core)| ~0.56 - 0.78 ms
| x86-64 native code (AVX) via [**Native Plugin**](https://docs.unity3d.com/Manual/NativePlugins.html) (single core)| ~0.32 - 0.51 ms

![](https://raw.githubusercontent.com/pm4rtx/Unity-FrustumCulling/master/Preview/Showcase.png)

## Fork

Despite the Burst compiler offering great performance, simplicity of use, portability and the ability to quickly parallelise
C# culling code via the job system to saturate all available cores, there was an interest in the source code for the native plugin,
which the original release of the sample didn't offer. This fork provides the assembly code placed under the [`Sources`](https://github.com/pm4rtx/Unity-FrustumCulling/tree/master/Sources)
directory. Building the assembly requires running `frustumcull.build.cmd` or `frustumcull.make` from the [`Scripts`](https://github.com/pm4rtx/Unity-FrustumCulling/tree/master/Scripts)
directory. To build the native plugin without any errors `PATH` system variable must contain paths to `nasm.exe` and
Visual Studio `lib.exe` and `link.exe`. Optionally, `nmake.exe` should be available when `.make` file is preferred.

## Native Plugin Performance

Below tables contain performance numbers for routines exposed through native plugin.

### Pre-warmed cache

|                                                       | Time (i7-4700 HQ)             | Throughput (i7-4700 HQ)|
|------------------------------------------------------:|:-----------------------------:|:-----------------------------:|
| `frustumcull_aosoa4_aabb_stream_simd4_sse2`           | ~0.33 ms                      | ~300 000 AABBs/ms             |
| `frustumcull_aosoa8_aabb_stream_simd8_avx`            | ~0.17 ms                      | ~590 000 AABBs/ms             |
| `frustumcull_aosoa4_aabb_stream_simd4_avx`            | ~0.30 ms                      | ~330 000 AABBs/ms             |
| `frustumcull_aosoa4_aabb_stream_simd8_avx`            | ~0.18 ms                      | ~550 000 AABBs/ms             |
| `frustumcull_extratests_aosoa4_aabb_stream_simd4_sse2`| ~0.40 ms                      | ~245 000 AABBs/ms             |
| `frustumcull_extratests_aosoa8_aabb_stream_simd8_avx` | ~0.19 ms                      | ~520 000 AABBs/ms             |
| `frustumcull_extratests_aosoa4_aabb_stream_simd4_avx` | ~0.35 ms                      | ~285 000 AABBs/ms             |
| `frustumcull_extratests_aosoa4_aabb_stream_simd8_avx` | ~0.21 ms                      | ~470 000 AABBs/ms             |

### Cold cache

|                                                       | Time (i7-4700 HQ)             | Throughput (i7-4700 HQ)|
|------------------------------------------------------:|:-----------------------------:|:-----------------------------:|
| `frustumcull_aosoa4_aabb_stream_simd4_sse2`           | ~0.35 ms                      | ~290 000 AABBs/ms             |
| `frustumcull_aosoa8_aabb_stream_simd8_avx`            | ~0.20 ms                      | ~500 000 AABBs/ms             |
| `frustumcull_aosoa4_aabb_stream_simd4_avx`            | ~0.32 ms                      | ~310 000 AABBs/ms             |
| `frustumcull_aosoa4_aabb_stream_simd8_avx`            | ~0.21 ms                      | ~470 000 AABBs/ms             |
| `frustumcull_extratests_aosoa4_aabb_stream_simd4_sse2`| ~0.42 ms                      | ~240 000 AABBs/ms             |
| `frustumcull_extratests_aosoa8_aabb_stream_simd8_avx` | ~0.23 ms                      | ~450 000 AABBs/ms             |
| `frustumcull_extratests_aosoa4_aabb_stream_simd4_avx` | ~0.36 ms                      | ~270 000 AABBs/ms             |
| `frustumcull_extratests_aosoa4_aabb_stream_simd8_avx` | ~0.24 ms                      | ~420 000 AABBs/ms             |

During the measurement, the data sets contained 100 000 bounding boxes arranged into array of structures of arrays (SoA)
containing four or eight boxes (depending on expected routine input format):

```c
typedef struct soa4_aabb
{
    float minX[4];
    float maxX[4];
    float minY[4];
    float maxY[4];
    float minZ[4];
    float maxZ[4];
} soa4_aabb;

void frustumcull_aosoa4_aabb_stream_simd4_sse2(u64 *out_mask, soa4_aabb const *soa_aabb, u32 soa_aabb_count);
void frustumcull_aosoa4_aabb_stream_simd4_avx(u64 *out_mask, soa4_aabb const *soa_aabb, u32 soa_aabb_count);
void frustumcull_aosoa4_aabb_stream_simd8_avx(u64 *out_mask, soa4_aabb const *soa_aabb, u32 soa_aabb_count);

void frustumcull_extratests_aosoa4_aabb_stream_simd4_sse2(u64 *out_mask, soa4_aabb const *soa_aabb, u32 soa_aabb_count);
void frustumcull_extratests_aosoa4_aabb_stream_simd4_avx(u64 *out_mask, soa4_aabb const *soa_aabb, u32 soa_aabb_count);
void frustumcull_extratests_aosoa4_aabb_stream_simd8_avx(u64 *out_mask, soa4_aabb const *soa_aabb, u32 soa_aabb_count);

typedef struct soa8_aabb
{
    float minX[8];
    float maxX[8];
    float minY[8];
    float maxY[8];
    float minZ[8];
    float maxZ[8];
} soa8_aabb;

void frustumcull_aosoa8_aabb_stream_simd8_avx(u64 *out_mask, soa8_aabb const *soa_aabb, u32 soa_aabb_count);
void frustumcull_extratests_aosoa8_aabb_stream_simd8_avx(u64 *out_mask, soa8_aabb const *soa_aabb, u32 soa_aabb_count);
```

## Package Dependencies

- com.unity.burst: `1.5.6`
- com.unity.collections: `0.0.9-preview.17`
- com.unity.jobs: `0.0.7-preview.10`
- com.unity.mathematics: `1.2.6`
- com.unity.package-manager-ui: `2.1.1`

## Tested Unity version

- 2017.3+ (NOTE: Burst and Mathematics packages don't work with this version, but native still can be used there)
- 2019.1+
