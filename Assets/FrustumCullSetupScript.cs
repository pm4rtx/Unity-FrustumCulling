using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;

#if UNITY_2019_1_OR_NEWER
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;

public static class NativeArrayExtensions
{
    public static unsafe System.IntPtr GetNativePtr<T>(this NativeArray<T> a) where T : struct
    {
        return new System.IntPtr(a.GetUnsafeReadOnlyPtr());
    }
}

#else
using UnityEngine.Collections;

public static class NativeArrayExtensions
{
    public static System.IntPtr GetNativePtr<T>(this NativeArray<T> a) where T : struct
    {
        return a.UnsafeReadOnlyPtr;
    }
}

#endif


public class FrustumCullSetupScript : MonoBehaviour
{
    const float kMinExtendX = 0.1f;
    const float kMinExtendY = 0.1f;
    const float kMinExtendZ = 0.1f;

    // NOTE: see this https://docs.unity3d.com/ScriptReference/Graphics.DrawMeshInstanced.html
    const uint kMaxMeshesPerDrawBatch = 1023;

    public enum JobType
    {
        ReferenceSerialOnly,
        NativePluginSimd4SSE2,
        NativePluginSimd4AVX,
        NativePluginSimd8AVX,
#if UNITY_2019_1_OR_NEWER
        MathematicsNoBurst,
        MathematicsBurstOptimized
#endif
    };

    struct PinnedArray<T>
    {
        public T []     Array;
        public GCHandle Handle;

        public static implicit operator T[] (PinnedArray<T> x)
        {
            return x.Array;
        }

        public System.IntPtr Address()
        {
            UnityEngine.Assertions.Assert.IsTrue(Handle.IsAllocated);
            return Handle.AddrOfPinnedObject();
        }

        public void Reserve(int NumElements)
        {
            if (Array != null && Array.Length < NumElements)
            {
                UnityEngine.Assertions.Assert.IsTrue(Handle.IsAllocated);
                Handle.Free();
                Array = null;
            }

            if (Array == null)
            {
                Array  = new T[NumElements];
                Handle = GCHandle.Alloc(Array, GCHandleType.Pinned);
            }
        }

        public void Release()
        {
            if (Array != null)
            {
                UnityEngine.Assertions.Assert.IsTrue(Handle.IsAllocated);
                Handle.Free();
                Array = null;
            }
        }
    }

    public Mesh UnitCubeMesh;
    public Material DefaultMaterial;

    [Range(1.0f, 1000.0f)]
    public float RandomDistanceH = 1000.0f;

    [Range(1.0f, 1000.0f)]
    public float RandomDistanceV = 50.0f;

    [Range(kMinExtendX, 100.0f)]
    public float RandomExtentLimitX = 10.0f;

    [Range(kMinExtendY, 100.0f)]
    public float RandomExtentLimitY = 10.0f;

    [Range(kMinExtendZ, 100.0f)]
    public float RandomExtentLimitZ = 10.0f;

    public bool RandomizeBoxes = true;

    [Range(16, 8192 << 4)]
    public int NumMeshTransforms = 16;
    private int PrevNumMeshTransforms = 0;
    private Matrix4x4 [] MeshTransforms;

    private NativeArray<byte> VisibilityMask;
    private NativeArray<BoxMinMaxSoA> BoxesMinMaxAoSoA;

    public bool TestFrustumCorners = false;

    public int NumMeshesToDraw = 0;

    PinnedArray<Vector4>    FrustumData;

    public JobType CullingJobType = JobType.ReferenceSerialOnly;
    public bool CullingJobParallel = false;

    private static Vector4 NormalizePlane(Vector4 Plane)
    {
        var LengthSquared = Plane.x * Plane.x + Plane.y * Plane.y + Plane.z * Plane.z;
        return Plane / LengthSquared;
    }

    private static Vector3 PointFrom3Planes(Vector4 Plane0, Vector4 Plane1, Vector4 Plane2)
    {
        Matrix4x4 M = Matrix4x4.identity;
        M.SetRow(0, (Vector4)(Vector3)Plane0);
        M.SetRow(1, (Vector4)(Vector3)Plane1);
        M.SetRow(2, (Vector4)(Vector3)Plane2);

        if (M.determinant != 0.0f)
        {
            var M2 = M.inverse;
            return M2.MultiplyVector(new Vector3(-Plane0.w, -Plane1.w, -Plane2.w));
        }
        return Vector3.zero;
    }

    private static void DebugDrawCross(Vector3 Point, Vector3 LateralNormal, float Size)
    {
        var offset0 = LateralNormal * Size;
        var offset1 = Vector3.up * Size;
        Debug.DrawLine(Point, Point + offset0, Color.green);
        Debug.DrawLine(Point - offset1, Point + offset1, Color.green);
    }

    private static Vector3 RayDirFromPlanes(Vector4 Plane0, Vector4 Plane1)
    {
        return Vector3.Normalize(Vector3.Cross((Vector3)Plane0, (Vector3)Plane1));
    }

#if UNITY_2019_1_OR_NEWER
    struct FrustumCullingJobData
    {
        [ReadOnly]
        public NativeSlice<float4> MinXSlice;
        [ReadOnly]
        public NativeSlice<float4> MaxXSlice;
        [ReadOnly]
        public NativeSlice<float4> MinYSlice;
        [ReadOnly]
        public NativeSlice<float4> MaxYSlice;
        [ReadOnly]
        public NativeSlice<float4> MinZSlice;
        [ReadOnly]
        public NativeSlice<float4> MaxZSlice;

        [WriteOnly]
        public NativeArray<byte> VisibilityMask;

        public float4 Frustum_Plane0;
        public float4 Frustum_Plane1;
        public float4 Frustum_Plane2;
        public float4 Frustum_Plane3;
        public float4 Frustum_Plane4;
        public float4 Frustum_Plane5;
        public float4 Frustum_xXyY;
        public float4 Frustum_zZzZ;

        public bool TestFrustumCorners;

        static private float4 MaxDotProduct(float4 MinX, float4 MaxX, float4 MinY, float4 MaxY, float4 MinZ, float4 MaxZ,float4 Plane)
        {
            return math.max(MinX * Plane.x, MaxX * Plane.x) +
                   math.max(MinY * Plane.y, MaxY * Plane.y) +
                   math.max(MinZ * Plane.z, MaxZ * Plane.z) +
                   Plane.wwww;
        }

        public void ComputeVisibility(int i)
        {
            var MinX = MinXSlice[i];
            var MaxX = MaxXSlice[i];
            var MinY = MinYSlice[i];
            var MaxY = MaxYSlice[i];
            var MinZ = MinZSlice[i];
            var MaxZ = MaxZSlice[i];

            uint4 Mask = 0;

            // box is fully outside of any frustum plane
            Mask |= math.asuint(MaxDotProduct(MinX, MaxX, MinY, MaxY, MinZ, MaxZ, Frustum_Plane0));
            Mask |= math.asuint(MaxDotProduct(MinX, MaxX, MinY, MaxY, MinZ, MaxZ, Frustum_Plane1));
            Mask |= math.asuint(MaxDotProduct(MinX, MaxX, MinY, MaxY, MinZ, MaxZ, Frustum_Plane2));
            Mask |= math.asuint(MaxDotProduct(MinX, MaxX, MinY, MaxY, MinZ, MaxZ, Frustum_Plane3));
            Mask |= math.asuint(MaxDotProduct(MinX, MaxX, MinY, MaxY, MinZ, MaxZ, Frustum_Plane4));
            Mask |= math.asuint(MaxDotProduct(MinX, MaxX, MinY, MaxY, MinZ, MaxZ, Frustum_Plane5));

            // frustum is fully outside of any box plane
            if (TestFrustumCorners)
            {
                Mask |= math.asuint(Frustum_xXyY.yyyy - MinX);
                Mask |= math.asuint(Frustum_xXyY.wwww - MinY);
                Mask |= math.asuint(Frustum_zZzZ.yyyy - MinZ);
                Mask |= math.asuint(MaxX - Frustum_xXyY.xxxx);
                Mask |= math.asuint(MaxY - Frustum_xXyY.zzzz);
                Mask |= math.asuint(MaxZ - Frustum_zZzZ.xxxx);
            }

            /*
            Mask = Mask >> 31;
            VisibilityMask[i] = (byte)((Mask.w << 3) | (Mask.z << 2) | (Mask.y << 1) | (Mask.x));
            /*/
            Mask = Mask & 0x80000000;
            VisibilityMask[i] = (byte)((Mask.w >> 28) | (Mask.z >> 29) | (Mask.y >> 30) | (Mask.x >> 31));
            /**/
        }
    }

    public interface ICullingJob
    {
        void RunParallelAndWait();
        void RunSerial();
    }

    struct FrustumCullBoxesAoSoAJob_NoBurst : Unity.Jobs.IJobParallelFor, ICullingJob
    {
        public FrustumCullingJobData JobData;

        public FrustumCullBoxesAoSoAJob_NoBurst(ref FrustumCullingJobData Data)
        {
            JobData = Data;
        }

        public void RunParallelAndWait()
        {
            this.Schedule(JobData.VisibilityMask.Length, 32).Complete();
        }

        public void RunSerial()
        {
            this.Run(JobData.VisibilityMask.Length);
        }

        public void Execute(int i)
        {
            JobData.ComputeVisibility(i);
        }
    }

    [Unity.Burst.BurstCompile]
    struct FrustumCullBoxesAoSoAJob : Unity.Jobs.IJobParallelFor, ICullingJob
    {
        public FrustumCullingJobData JobData;

        public FrustumCullBoxesAoSoAJob(ref FrustumCullingJobData Data)
        {
            JobData = Data;
        }

        public void RunParallelAndWait()
        {
            this.Schedule(JobData.VisibilityMask.Length, 32).Complete();
        }

        public void RunSerial()
        {
            this.Run(JobData.VisibilityMask.Length);
        }

        public void Execute(int i)
        {
            JobData.ComputeVisibility(i);
        }
    }

#endif

    public struct BoxMinMaxSoA
    {
        public Vector4 MinX;
        public Vector4 MaxX;
        public Vector4 MinY;
        public Vector4 MaxY;
        public Vector4 MinZ;
        public Vector4 MaxZ;

        static private Vector3 RandomCenter(Vector2 RandomDistance)
        {
            var center = Vector3.zero;
            center.x = UnityEngine.Random.value * RandomDistance.x * 2.0f - RandomDistance.x;
            center.z = UnityEngine.Random.value * RandomDistance.x * 2.0f - RandomDistance.x;
            center.y = UnityEngine.Random.value * RandomDistance.y * 2.0f - RandomDistance.y;
            return center;
        }

        static private Vector3 RandomExtent(Vector3 RandomExtendLimit)
        {
            var extent = Vector3.zero;
            extent.x = RandomExtendLimit.x + UnityEngine.Random.value * (RandomExtendLimit.x - kMinExtendX);
            extent.y = RandomExtendLimit.y + UnityEngine.Random.value * (RandomExtendLimit.y - kMinExtendY);
            extent.z = RandomExtendLimit.z + UnityEngine.Random.value * (RandomExtendLimit.z - kMinExtendZ);
            return extent;
        }

        static public BoxMinMaxSoA MakeRandom(Vector3 RandomExtendLimit, Vector2 RandomDistance)
        {
            var b = new BoxMinMaxSoA();
            var c0 = RandomCenter(RandomDistance);
            var e0 = RandomExtent(RandomExtendLimit);

            var c1 = RandomCenter(RandomDistance);
            var e1 = RandomExtent(RandomExtendLimit);

            var c2 = RandomCenter(RandomDistance);
            var e2 = RandomExtent(RandomExtendLimit);

            var c3 = RandomCenter(RandomDistance);
            var e3 = RandomExtent(RandomExtendLimit);

            b.MinX = new Vector4(c0[0] - e0[0], c1[0] - e1[0], c2[0] - e2[0], c3[0] - e3[0]);
            b.MaxX = new Vector4(c0[0] + e0[0], c1[0] + e1[0], c2[0] + e2[0], c3[0] + e3[0]);
            b.MinY = new Vector4(c0[1] - e0[1], c1[1] - e1[1], c2[1] - e2[1], c3[1] - e3[1]);
            b.MaxY = new Vector4(c0[1] + e0[1], c1[1] + e1[1], c2[1] + e2[1], c3[1] + e3[1]);
            b.MinZ = new Vector4(c0[2] - e0[2], c1[2] - e1[2], c2[2] - e2[2], c3[2] - e3[2]);
            b.MaxZ = new Vector4(c0[2] + e0[2], c1[2] + e1[2], c2[2] + e2[2], c3[2] + e3[2]);
            return b;
        }

        public void ToTransformMatrix(ref Matrix4x4[] OutTransforms, int OutTransformIdx, int ComponentIdx)
        {
            var Min = new Vector3(MinX[ComponentIdx], MinY[ComponentIdx], MinZ[ComponentIdx]);
            var Max = new Vector3(MaxX[ComponentIdx], MaxY[ComponentIdx], MaxZ[ComponentIdx]);
            OutTransforms[OutTransformIdx].SetTRS((Max + Min) * 0.5f, Quaternion.identity, (Max - Min) * 0.5f);
        }
    }

    static private void FilterOutCulledBoxes(
        ref int InOutSoAPacketStart,
        ref NativeArray<BoxMinMaxSoA> InBoxesMinMaxAoSoA,
        ref NativeArray<byte> InVisMask,
        ref int OutTransformCount,
        ref Matrix4x4[] OutTransforms,
        bool BytePerPacket)
    {
        int TransformCount = 0;
        var NumSoAPackets = InBoxesMinMaxAoSoA.Length;
        if (InOutSoAPacketStart >= NumSoAPackets)
        {
            return;
        }

        UnityEngine.Assertions.Assert.AreEqual(NumSoAPackets, InVisMask.Length);
        UnityEngine.Assertions.Assert.AreEqual(kMaxMeshesPerDrawBatch, (uint)OutTransforms.Length);

        int i;
        for (i = InOutSoAPacketStart; i < NumSoAPackets; ++i)
        {
            var Byte = BytePerPacket ? InVisMask[i] : (InVisMask[i >> 1] >> ((i & 0x1) << 2));

            if (Byte != 0xf)
            {
                int TransformCountPerPacket = 0;

                if ((Byte & 0x1) == 0) ++TransformCountPerPacket;
                if ((Byte & 0x2) == 0) ++TransformCountPerPacket;
                if ((Byte & 0x4) == 0) ++TransformCountPerPacket;
                if ((Byte & 0x8) == 0) ++TransformCountPerPacket;

                // NOTE: if all transform from current SoA box packet can't fit into the array of matrices, end the loop
                // and return from the function giving a chance the calling code to process returned matrices
                if (TransformCount + TransformCountPerPacket > kMaxMeshesPerDrawBatch)
                {
                    break;
                }

                var b = InBoxesMinMaxAoSoA[i];

                if ((Byte & 0x1) == 0)
                    b.ToTransformMatrix(ref OutTransforms, TransformCount ++, 0);

                if ((Byte & 0x2) == 0)
                    b.ToTransformMatrix(ref OutTransforms, TransformCount ++, 1);

                if ((Byte & 0x4) == 0)
                    b.ToTransformMatrix(ref OutTransforms, TransformCount ++, 2);

                if ((Byte & 0x8) == 0)
                    b.ToTransformMatrix(ref OutTransforms, TransformCount ++, 3);
            }
        }
        InOutSoAPacketStart = i;
        OutTransformCount = TransformCount;
    }


    private static Vector4 FrustumOutsideAABB(ref BoxMinMaxSoA Box, Vector3 FrustumAABBMin, Vector3 FrustumAABBMax)
    {
        var r0 = new Vector4(FrustumAABBMax.x, FrustumAABBMax.x, FrustumAABBMax.x, FrustumAABBMax.x) - Box.MinX;
        var r1 = new Vector4(FrustumAABBMax.y, FrustumAABBMax.y, FrustumAABBMax.y, FrustumAABBMax.y) - Box.MinY;
        var r2 = new Vector4(FrustumAABBMax.z, FrustumAABBMax.z, FrustumAABBMax.z, FrustumAABBMax.z) - Box.MinZ;
        var r3 = Box.MaxX - new Vector4(FrustumAABBMin.x, FrustumAABBMin.x, FrustumAABBMin.x, FrustumAABBMin.x);
        var r4 = Box.MaxY - new Vector4(FrustumAABBMin.y, FrustumAABBMin.y, FrustumAABBMin.y, FrustumAABBMin.y);
        var r5 = Box.MaxZ - new Vector4(FrustumAABBMin.z, FrustumAABBMin.z, FrustumAABBMin.z, FrustumAABBMin.z);
        return Vector4.Min(Vector4.Min(Vector4.Min(r0, r1), Vector4.Min(r2, r3)), Vector4.Min(r4, r5));
    }

    private static Vector4 MaxDotProductPlaneAABB(ref BoxMinMaxSoA Box, Vector4 Plane)
    {
        return Vector4.Max(Box.MinX * Plane.x, Box.MaxX * Plane.x) +
               Vector4.Max(Box.MinY * Plane.y, Box.MaxY * Plane.y) +
               Vector4.Max(Box.MinZ * Plane.z, Box.MaxZ * Plane.z) +
               new Vector4(Plane.w, Plane.w, Plane.w, Plane.w);
    }

    static private void CullBoxesAoSoA_Default(
        ref NativeArray<byte> OutVisibilityMask,
        ref NativeArray<BoxMinMaxSoA> InBoxesMinMaxAoSoA,
        ref Vector4[] InFrustumData,
        Vector3 FrustumAABBMax,
        Vector3 FrustumAABBMin,
        bool TestFrustumCorners)
    {
        int NumSoAPackets = InBoxesMinMaxAoSoA.Length;

        for (int i = 0; i < NumSoAPackets; ++i)
        {
            var BoxMinMax = InBoxesMinMaxAoSoA[i];

            var DotsL = MaxDotProductPlaneAABB(ref BoxMinMax, InFrustumData[0]);
            var DotsR = MaxDotProductPlaneAABB(ref BoxMinMax, InFrustumData[1]);
            var DotsT = MaxDotProductPlaneAABB(ref BoxMinMax, InFrustumData[2]);
            var DotsB = MaxDotProductPlaneAABB(ref BoxMinMax, InFrustumData[3]);
            var DotsN = MaxDotProductPlaneAABB(ref BoxMinMax, InFrustumData[4]);
            var DotsF = MaxDotProductPlaneAABB(ref BoxMinMax, InFrustumData[5]);

            var R = Vector4.Min(Vector4.Min(DotsL, DotsR), Vector4.Min(Vector4.Min(DotsT, DotsB), Vector4.Min(DotsF, DotsN)));

            if (TestFrustumCorners)
            {
                R  = Vector4.Min(R, FrustumOutsideAABB(ref BoxMinMax, FrustumAABBMin, FrustumAABBMax));
            }


            int Mask = (R.w < 0.0f ? 0x8 : 0x0)
                     | (R.z < 0.0f ? 0x4 : 0x0)
                     | (R.y < 0.0f ? 0x2 : 0x0)
                     | (R.x < 0.0f ? 0x1 : 0x0);

            // init mask when index is even, update mask when it's odd
            if ((i & 0x1) == 0)
                OutVisibilityMask[i >> 1] = (byte)Mask;
            else
                OutVisibilityMask[i >> 1] |= (byte)(Mask << 4);
        }
    }

    [DllImport("UnityNativePlugin-frustumcull")]
    private static extern void frustumcull_aosoa4_aabb_stream_simd4_sse2(
        System.IntPtr OutVisibilityMask,
        System.IntPtr BoxesMinMaxAoSoA,
        System.IntPtr FrustumData,
        int count);

    [DllImport("UnityNativePlugin-frustumcull")]
    private static extern void frustumcull_aosoa4_aabb_stream_simd4_avx(
        System.IntPtr OutVisibilityMask,
        System.IntPtr BoxesMinMaxAoSoA,
        System.IntPtr FrustumData,
        int count);

    [DllImport("UnityNativePlugin-frustumcull")]
    private static extern void frustumcull_aosoa4_aabb_stream_simd8_avx(
        System.IntPtr OutVisibilityMask,
        System.IntPtr BoxesMinMaxAoSoA,
        System.IntPtr FrustumData,
        int count);

    [DllImport("UnityNativePlugin-frustumcull")]
    private static extern void frustumcull_extratests_aosoa4_aabb_stream_simd4_sse2(
        System.IntPtr OutVisibilityMask,
        System.IntPtr BoxesMinMaxAoSoA,
        System.IntPtr FrustumData,
        int count);

    [DllImport("UnityNativePlugin-frustumcull")]
    private static extern void frustumcull_extratests_aosoa4_aabb_stream_simd4_avx(
        System.IntPtr OutVisibilityMask,
        System.IntPtr BoxesMinMaxAoSoA,
        System.IntPtr FrustumData,
        int count);

    [DllImport("UnityNativePlugin-frustumcull")]
    private static extern void frustumcull_extratests_aosoa4_aabb_stream_simd8_avx(
        System.IntPtr OutVisibilityMask,
        System.IntPtr BoxesMinMaxAoSoA,
        System.IntPtr FrustumData,
        int count);

    void Start()
    {
        FrustumData.Reserve(8);

        if (BoxesMinMaxAoSoA.IsCreated)
        {
            Debug.Log("BoxesMinMaxAoSoA: " + BoxesMinMaxAoSoA);
            BoxesMinMaxAoSoA.Dispose();
        }
        MeshTransforms = new Matrix4x4[kMaxMeshesPerDrawBatch];
    }

    void OnDisable()
    {
        if (BoxesMinMaxAoSoA.IsCreated)
            BoxesMinMaxAoSoA.Dispose();

        if (VisibilityMask.IsCreated)
            VisibilityMask.Dispose();

        FrustumData.Release();
    }

    // Update is called once per frame
    void Update()
    {
        var ProjMatrix = Camera.main.projectionMatrix;
        var ViewMatrix = Camera.main.worldToCameraMatrix;

        var ViewProjMatrix = ProjMatrix * ViewMatrix;

        var Row3 = ViewProjMatrix.GetRow(3);
        var Row0 = ViewProjMatrix.GetRow(0);
        var Row1 = ViewProjMatrix.GetRow(1);
        var Row2 = ViewProjMatrix.GetRow(2);

        var PlaneL = NormalizePlane(Row3 + Row0);
        var PlaneR = NormalizePlane(Row3 - Row0);
        var PlaneT = NormalizePlane(Row3 - Row1);
        var PlaneB = NormalizePlane(Row3 + Row1);

        var PlaneN = NormalizePlane(Row3 + Row2);
        var PlaneF = NormalizePlane(Row3 - Row2);

        var PointNTL = PointFrom3Planes(PlaneN, PlaneT, PlaneL);
        var PointNTR = PointFrom3Planes(PlaneN, PlaneT, PlaneR);
        var PointNBL = PointFrom3Planes(PlaneN, PlaneB, PlaneL);
        var PointNBR = PointFrom3Planes(PlaneN, PlaneB, PlaneR);

        //DebugDrawCross(PointNTL, RayDirFromPlanes(PlaneT, PlaneL), 0.3f);
        //DebugDrawCross(PointNTR, RayDirFromPlanes(PlaneR, PlaneT), 0.3f);
        //DebugDrawCross(PointNBL, RayDirFromPlanes(PlaneL, PlaneB), 0.3f);
        //DebugDrawCross(PointNBR, RayDirFromPlanes(PlaneB, PlaneR), 0.3f);

        var PointFTL = PointFrom3Planes(PlaneF, PlaneT, PlaneL);
        var PointFTR = PointFrom3Planes(PlaneF, PlaneT, PlaneR);
        var PointFBL = PointFrom3Planes(PlaneF, PlaneB, PlaneL);
        var PointFBR = PointFrom3Planes(PlaneF, PlaneB, PlaneR);

        var FrustumAABBMin =
            Vector3.Min(Vector3.Min(Vector3.Min(PointFTL, PointFTR), Vector3.Min(PointFBL, PointFBR)),
                        Vector3.Min(Vector3.Min(PointNTL, PointNTR), Vector3.Min(PointNBL, PointNBR)));

        var FrustumAABBMax =
            Vector3.Max(Vector3.Max(Vector3.Max(PointFTL, PointFTR), Vector3.Max(PointFBL, PointFBR)),
                        Vector3.Max(Vector3.Max(PointNTL, PointNTR), Vector3.Max(PointNBL, PointNBR)));


        //DebugDrawCross(PointFTL, -RayDirFromPlanes(PlaneT, PlaneL), 1.3f);
        //DebugDrawCross(PointFTR, -RayDirFromPlanes(PlaneR, PlaneT), 1.3f);
        //DebugDrawCross(PointFBL, -RayDirFromPlanes(PlaneL, PlaneB), 1.3f);
        //DebugDrawCross(PointFBR, -RayDirFromPlanes(PlaneB, PlaneR), 1.3f);

        FrustumData.Array[0] = PlaneL;
        FrustumData.Array[1] = PlaneR;
        FrustumData.Array[2] = PlaneT;
        FrustumData.Array[3] = PlaneB;
        FrustumData.Array[4] = PlaneN;
        FrustumData.Array[5] = PlaneF;
        FrustumData.Array[6] = new Vector4(FrustumAABBMin.x, FrustumAABBMax.x, FrustumAABBMin.y, FrustumAABBMax.y);
        FrustumData.Array[7] = new Vector4(FrustumAABBMin.z, FrustumAABBMax.z, FrustumAABBMin.z, FrustumAABBMax.z);

        int NumSoAPackets = (NumMeshTransforms + 3) >> 2;
        NumMeshTransforms = NumSoAPackets << 2;

        if (PrevNumMeshTransforms != NumMeshTransforms)
        {
            Debug.Log("Re-allocate AABB data because of size change.");

            if (VisibilityMask.IsCreated)
                VisibilityMask.Dispose();

            if (BoxesMinMaxAoSoA.IsCreated)
                BoxesMinMaxAoSoA.Dispose();

#if UNITY_2019_1_OR_NEWER
            BoxesMinMaxAoSoA = new NativeArray<BoxMinMaxSoA>(NumSoAPackets, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            VisibilityMask = new NativeArray<byte>(NumSoAPackets, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
#else
            BoxesMinMaxAoSoA = new NativeArray<BoxMinMaxSoA>(NumSoAPackets, Allocator.Persistent);
            VisibilityMask = new NativeArray<byte>(NumSoAPackets, Allocator.Persistent);
#endif
            RandomizeBoxes = true;
        }
        PrevNumMeshTransforms = NumMeshTransforms;

        if (RandomizeBoxes)
        {
            UnityEngine.Random.InitState((int)(Time.time * 1000.0f));
            var RandomExtendLimit = new Vector3(RandomExtentLimitX, RandomExtentLimitY, RandomExtentLimitZ);
            var RandomDistance = new Vector2(RandomDistanceH, RandomDistanceV);
            for (int i = 0; i < NumSoAPackets; ++i)
            {
                BoxesMinMaxAoSoA[i] = BoxMinMaxSoA.MakeRandom(RandomExtendLimit, RandomDistance);
            }
            RandomizeBoxes = false;
        }

        if (CullingJobType == JobType.ReferenceSerialOnly)
        {
            CullingJobParallel = false;
            UnityEngine.Profiling.Profiler.BeginSample("CullBoxesAoSoA_Default");
            CullBoxesAoSoA_Default(
                ref VisibilityMask,
                ref BoxesMinMaxAoSoA,
                ref FrustumData.Array,
                FrustumAABBMax,
                FrustumAABBMin,
                TestFrustumCorners);
            UnityEngine.Profiling.Profiler.EndSample();
        }
        else if (CullingJobType >= JobType.NativePluginSimd4SSE2 && CullingJobType <= JobType.NativePluginSimd8AVX)
        {
            CullingJobParallel = false;
            UnityEngine.Profiling.Profiler.BeginSample("UnityNativePlugin_FrustumCullNative");
            if (TestFrustumCorners)
            {
                if (CullingJobType == JobType.NativePluginSimd4SSE2)
                    frustumcull_extratests_aosoa4_aabb_stream_simd4_sse2(VisibilityMask.GetNativePtr(), BoxesMinMaxAoSoA.GetNativePtr(), FrustumData.Address(), NumSoAPackets);
                else if (CullingJobType == JobType.NativePluginSimd4AVX)
                    frustumcull_extratests_aosoa4_aabb_stream_simd4_avx(VisibilityMask.GetNativePtr(), BoxesMinMaxAoSoA.GetNativePtr(), FrustumData.Address(), NumSoAPackets);
                else if (CullingJobType == JobType.NativePluginSimd8AVX)
                    frustumcull_extratests_aosoa4_aabb_stream_simd8_avx(VisibilityMask.GetNativePtr(), BoxesMinMaxAoSoA.GetNativePtr(), FrustumData.Address(), NumSoAPackets);
            }
            else
            {
                if (CullingJobType == JobType.NativePluginSimd4SSE2)
                    frustumcull_aosoa4_aabb_stream_simd4_sse2(VisibilityMask.GetNativePtr(), BoxesMinMaxAoSoA.GetNativePtr(), FrustumData.Address(), NumSoAPackets);
                else if (CullingJobType == JobType.NativePluginSimd4AVX)
                    frustumcull_aosoa4_aabb_stream_simd4_avx(VisibilityMask.GetNativePtr(), BoxesMinMaxAoSoA.GetNativePtr(), FrustumData.Address(), NumSoAPackets);
                else if (CullingJobType == JobType.NativePluginSimd8AVX)
                    frustumcull_aosoa4_aabb_stream_simd8_avx(VisibilityMask.GetNativePtr(), BoxesMinMaxAoSoA.GetNativePtr(), FrustumData.Address(), NumSoAPackets);
            }
            UnityEngine.Profiling.Profiler.EndSample();
        }
#if UNITY_2019_1_OR_NEWER
        else
        {
            ICullingJob job;

            var BoxMinMaxAoSoASlice = BoxesMinMaxAoSoA.Slice();

            var JobData = new FrustumCullingJobData
            {
                MinXSlice = BoxMinMaxAoSoASlice.SliceWithStride<float4>(0),
                MaxXSlice = BoxMinMaxAoSoASlice.SliceWithStride<float4>(16),
                MinYSlice = BoxMinMaxAoSoASlice.SliceWithStride<float4>(32),
                MaxYSlice = BoxMinMaxAoSoASlice.SliceWithStride<float4>(48),
                MinZSlice = BoxMinMaxAoSoASlice.SliceWithStride<float4>(64),
                MaxZSlice = BoxMinMaxAoSoASlice.SliceWithStride<float4>(80),
                Frustum_Plane0 = FrustumData.Array[0],
                Frustum_Plane1 = FrustumData.Array[1],
                Frustum_Plane2 = FrustumData.Array[2],
                Frustum_Plane3 = FrustumData.Array[3],
                Frustum_Plane4 = FrustumData.Array[4],
                Frustum_Plane5 = FrustumData.Array[5],
                Frustum_xXyY = FrustumData.Array[6],
                Frustum_zZzZ = FrustumData.Array[7],
                VisibilityMask = VisibilityMask,
                TestFrustumCorners = TestFrustumCorners
            };

            if (CullingJobType == JobType.MathematicsBurstOptimized)
                job = new FrustumCullBoxesAoSoAJob(ref JobData);
            else
                job = new FrustumCullBoxesAoSoAJob_NoBurst(ref JobData);

            if (CullingJobParallel)
            {
                UnityEngine.Profiling.Profiler.BeginSample("FrustumCullBoxesAoSoA_Parallel");
                job.RunParallelAndWait();
                UnityEngine.Profiling.Profiler.EndSample();
            }
            else
            {
                UnityEngine.Profiling.Profiler.BeginSample("FrustumCullBoxesAoSoA_Serial");
                job.RunSerial();
                UnityEngine.Profiling.Profiler.EndSample();
            }
        }
#endif
    }

    public void LateUpdate()
    {
        if (UnitCubeMesh && DefaultMaterial)
        {
            DefaultMaterial.enableInstancing = true;

            int SoAPacketStart = 0;
            NumMeshesToDraw = 0;

            while (SoAPacketStart != BoxesMinMaxAoSoA.Length)
            {
                int TransformsCount = 0;

                UnityEngine.Profiling.Profiler.BeginSample("FilterOutCulledBoxes");
#if UNITY_2019_1_OR_NEWER
                if (CullingJobType == JobType.MathematicsNoBurst || CullingJobType == JobType.MathematicsBurstOptimized)
                    FilterOutCulledBoxes(ref SoAPacketStart, ref BoxesMinMaxAoSoA, ref VisibilityMask, ref TransformsCount, ref MeshTransforms, true);
                else
#endif
                    FilterOutCulledBoxes(ref SoAPacketStart, ref BoxesMinMaxAoSoA, ref VisibilityMask, ref TransformsCount, ref MeshTransforms, false);
                UnityEngine.Profiling.Profiler.EndSample();

                Graphics.DrawMeshInstanced(
                     UnitCubeMesh
                    ,0
                    ,DefaultMaterial
                    ,MeshTransforms
                    ,TransformsCount
                    ,null
                    ,ShadowCastingMode.Off
                    ,false
                    ,0
                    ,null
#if UNITY_2019_1_OR_NEWER
                    ,LightProbeUsage.Off
                    ,null
#endif
                );
                NumMeshesToDraw += TransformsCount;
            }
        }
    }
}
