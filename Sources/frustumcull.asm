; frustumcull.asm -- SSE2 / AVX accelerated AABB/view frustum culling routine
;                    no warranty implied; use at your own risk
;
; BUILDING:
;   Building this file into .obj file should be done via NASM.
;   It's possible to define the name for the culling function and choose options
;   for it, similar to GPU shader, so multiple functions could be built and later
;   linked together into the library or executable
;
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj -DUSE_AVX=1
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj -DUSE_AVX=1 -DSIMD_SIZE=4
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj -DUSE_AVX=1 -DUSE_AOSOA4_INPUT=1
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj -DUSE_EXTRATESTS=1
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj -DUSE_EXTRATESTS=1 -DUSE_AVX=1
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -DSIMD_SIZE=4
;   nasm -DFUNCTION_NAME=fname -f win64 -Oxv frustumcull.asm -o fname.obj -DUSE_EXTRATESTS=1 -DUSE_AVX=1 -DUSE_AOSOA4_INPUT=1
;
;
; LICENSE:
;   This software is distributed under two licenses. See notice at end of the file.

section .text

%macro IACA_START 0
    mov ebx, 111
    db 0x64, 0x67, 0x90
%endmacro

%macro IACA_END 0
    mov ebx, 222
    db 0x64, 0x67, 0x90
%endmacro

%ifndef USE_AVX
    %define USE_AVX 0
%elif USE_AVX != 0 && USE_AVX != 1
    %fatal "USE_AVX macro must be set to 1 or 0"
%endif

%ifndef SIMD_SIZE
    %if USE_AVX
        %define SIMD_SIZE 8
    %else
        %define SIMD_SIZE 4
    %endif
%elif SIMD_SIZE != 4 && SIMD_SIZE != 8
    %fatal "SIMD_SIZE macro must be set to 4 or 8"
%endif

%if SIMD_SIZE == 4
    %define SimdName xmm
%elif SIMD_SIZE == 8
    %define SimdName ymm
%endif

%ifndef USE_EXTRATESTS
    %define USE_EXTRATESTS 0
%endif

%ifndef USE_AOSOA4_INPUT
    %if SIMD_SIZE == 4
        %define USE_AOSOA4_INPUT 1
    %elif SIMD_SIZE == 8
        %define USE_AOSOA4_INPUT 0
    %endif
%elif USE_AOSOA4_INPUT != 0 && USE_AOSOA4_INPUT != 1
    %fatal "USE_AOSOA4_INPUT macro must be set to 1 or 0"
%elif USE_AOSOA4_INPUT == 0 && SIMD_SIZE == 4
    %fatal "USE_AOSOA4_INPUT must be set to 1 or undefined when SIMD_SIZE == 4"
%endif

%if USE_AVX
    %define simd_op_f32(name) v %+ name %+ps
    %define simd_xop_f32(name) v %+ name %+ps

    ; create vcmpgtps for convinience
    %define vcmpgtps vcmpnleps
%else
    %define simd_op_f32(name) name %+ ps
    %define simd_xop_f32(name) x %+ name %+ps

    ; create cmpgtps for convinience
    %define cmpgtps cmpnleps

    ; artificially define eXtra-arg op
    %macro xmulps 3
        movaps %1, %2
        mulps %1, %3
    %endmacro

%endif

%macro simd_broadcast_m32 2
    %if USE_AVX
        vbroadcastss %1, %2
    %else
        movss %1, %2
        shufps %1, %1, 0
    %endif
%endmacro


%define Simd(SimdIdx) SimdName %+ SimdIdx
%define SimdByteSize (SIMD_SIZE * 4)

%macro Broadcast4FloatsFromMemTo4Simds 5

    simd_broadcast_m32 %1, [%5]
    simd_broadcast_m32 %2, [%5 +  4]
    simd_broadcast_m32 %3, [%5 +  8]
    simd_broadcast_m32 %4, [%5 + 12]

%endmacro

; saves XMM6-XMM15 or YMM6-YMM15 registers consecutively on stack,
; XMM6/YMM6 is located at [RSP - bytesize(reg_type)]
; XMM15/YMM15 is located at [RSP - bytesize(reg_type) * 10]
%macro SimdSaveNonVolatileRegistersToStack 0

    ; round down the address stored in RSP to align stack pointer by growing stack a bit
    and rsp, -SimdByteSize

    ; allocate stack memory enough to save all simd registers
    add rsp, -SimdByteSize * 10

    %assign SimdIdx 6
    %rep 10
    simd_op_f32(mova) [rsp + (15 - SimdIdx) *  SimdByteSize], Simd(SimdIdx)
    %assign SimdIdx SimdIdx + 1
    %endrep

%endmacro

; loads XMM6-XMM15 or YMM6-YMM15 registers from stack stack, see SimdSaveNonVolatileRegistersToStack
%macro SimdReLoadNonVolatileRegistersFromStack 0

    %assign SimdIdx 6
    %rep 10
    simd_op_f32(mova) Simd(SimdIdx), [rsp + (15 - SimdIdx) *  SimdByteSize]
    %assign SimdIdx SimdIdx + 1
    %endrep

    ; deallocate memory from the stack
    add rsp, SimdByteSize * 10

%endmacro

; initializes RSI and RDI
; %1 - dst address
; %2 - src address
; %3 - the number of work items
; %4 - the size of one work item
%macro StartSrcDstLoop 4

    ; save RSI, RDI, RCX registers as they are used as working register
    push rdi
    push rsi

    ; move dst and src addresses to appropriate registers, move the number of work items into RCX
    mov rdi, %1
    mov rsi, %2
    mov rcx, %3

    ; calculate the byte size of the whole working set by multiplying the number of work items,
    ; stored in RCX by the size of the single work item
    imul rcx, %4

    ; compute the end address of the source and desination sequences of data
    add rdi, rcx
    add rsi, rcx

    ; make byte size of the working set negative to reduce the size
    neg rcx

%endmacro

; restores RSI and RDI registers
%macro EndSrcDstLoop 0

    pop rsi
    pop rdi

%endmacro

; %1 - Simd register accumulating the result
; %2 - The expression containing the address of the plane to test againt
; Expects  Simd(10), Simd(12), Simd(14) contains maxX, maxY and maxZ respectively for 4 aabbs
; Expects  Simd(11), Simd(13), Simd(15) contains minX, minY and minZ respectively for 4 aabbs
%macro IsBoxFullyOutsidePlane_MaxSum 2

    ; load plane from memory (address stored in %2 register) and broadcast each component to SIMD registers
    ; Simd0 - XXXX, Simd1 - YYYY, Simd2 - ZZZZ, Simd6 - WWWW
    Broadcast4FloatsFromMemTo4Simds Simd(0), Simd(1), Simd(2), Simd(6), %2

    ; plane.xyz * min.xyz;
    simd_xop_f32(mul) Simd(3), Simd(0), Simd(15)
    simd_xop_f32(mul) Simd(4), Simd(1), Simd(13)
    simd_xop_f32(mul) Simd(5), Simd(2), Simd(11)

    ; plane.xyz * max.xyz;
    simd_op_f32(mul) Simd(0), Simd(14)
    simd_op_f32(mul) Simd(1), Simd(12)
    simd_op_f32(mul) Simd(2), Simd(10)

    ; max(plane.xyz * max.xyz, plane.xyz * min.xyz)
    simd_op_f32(max)  Simd(0), Simd(3)
    simd_op_f32(max)  Simd(1), Simd(4)
    simd_op_f32(max)  Simd(2), Simd(5)

    simd_op_f32(add)  Simd(6), Simd(0)
    simd_op_f32(add)  Simd(1), Simd(2)
    simd_op_f32(add)  Simd(6), Simd(1)

    ; if the lane value < 0, then corresponding bounding box is outside the the plane
    ; use `or` operation to update the result of all test (only upper bit is interesting for each lane)
    simd_op_f32(or) %1, Simd(6)

%endmacro

; Default DLL Entry point doing nothing
;-----------------------------------------------------------------------------------------------------------------------
%ifndef FUNCTION_NAME
global _DllMainCRTStartup

_DllMainCRTStartup:

    mov al, 1
    ret

%else

; Culls a stream of bounding boxes
;-----------------------------------------------------------------------------------------------------------------------
global FUNCTION_NAME
export FUNCTION_NAME
FUNCTION_NAME:

    push rbp
    mov  rbp, rsp

%if USE_AOSOA4_INPUT && SIMD_SIZE == 8
    ; if SIMD4 packets is used then we are going to treat 2 simd4 packets as one simd8 packet, so adjust the number of
    ; packets accordingly
    shr r9, 1
%endif

    SimdSaveNonVolatileRegistersToStack

    StartSrcDstLoop rcx, rdx, r9, SimdByteSize * 6

    ; remove effect of the StartSrcDstLoop to restore RDI
    add rdi, rcx
    ; clear RAX
    xor rax, rax

    mov rdx, rcx
    ; set RCX counter to the default value
    mov rcx, -64

.loop:

    ;IACA_START
    ;-------------------------------------------------------------
    ; Zero the register to accumulate the mask of all tests
    simd_op_f32(xor) Simd(9), Simd(9)

    simd_op_f32(movu) Simd(15), [rsi + rdx]
    simd_op_f32(movu) Simd(14), [rsi + rdx + SimdByteSize * 1]
    simd_op_f32(movu) Simd(13), [rsi + rdx + SimdByteSize * 2]
    simd_op_f32(movu) Simd(12), [rsi + rdx + SimdByteSize * 3]
    simd_op_f32(movu) Simd(11), [rsi + rdx + SimdByteSize * 4]
    simd_op_f32(movu) Simd(10), [rsi + rdx + SimdByteSize * 5]

%if USE_AOSOA4_INPUT && SIMD_SIZE == 8
    vperm2f128 Simd(0), Simd(15), Simd(12), 0x20
    vperm2f128 Simd(1), Simd(15), Simd(12), 0x31

    vperm2f128 Simd(2), Simd(14), Simd(11), 0x20
    vperm2f128 Simd(3), Simd(14), Simd(11), 0x31

    vperm2f128 Simd(4), Simd(13), Simd(10), 0x20
    vperm2f128 Simd(5), Simd(13), Simd(10), 0x31

    vmovaps Simd(15), Simd(0)
    vmovaps Simd(14), Simd(1)

    vmovaps Simd(13), Simd(2)
    vmovaps Simd(12), Simd(3)

    vmovaps Simd(11), Simd(4)
    vmovaps Simd(10), Simd(5)
%endif

    ; check if boxes outside the frustum in SoA way,
    ; result of each of N tests (depending on SIMD size) is stored in the upper bit of the 32-bit lane of 9 register
    IsBoxFullyOutsidePlane_MaxSum Simd(9), r8
    IsBoxFullyOutsidePlane_MaxSum Simd(9), r8 + 16
    IsBoxFullyOutsidePlane_MaxSum Simd(9), r8 + 32
    IsBoxFullyOutsidePlane_MaxSum Simd(9), r8 + 48
    IsBoxFullyOutsidePlane_MaxSum Simd(9), r8 + 64
    IsBoxFullyOutsidePlane_MaxSum Simd(9), r8 + 80

    ; check if the frustum is outside of 4 boxes
%if USE_EXTRATESTS
    simd_broadcast_m32 Simd(0), [r8 + 96 + 0]
    simd_broadcast_m32 Simd(1), [r8 + 96 + 4]

    ; check if:
    ;       box.max_x < frustum.min_x
    ;       frustum.max_x < box_min_x
    simd_op_f32(cmpgt) Simd(0), Simd(14)
    simd_op_f32(cmplt) Simd(1), Simd(15)

    ; accumulate upper bits storing sign
    simd_op_f32(or) Simd(9), Simd(0)
    simd_op_f32(or) Simd(9), Simd(1)

    simd_broadcast_m32 Simd(2), [r8 + 96 +  8]
    simd_broadcast_m32 Simd(3), [r8 + 96 + 12]

    simd_op_f32(cmpgt) Simd(2), Simd(12)
    simd_op_f32(cmplt) Simd(3), Simd(13)

    simd_op_f32(or) Simd(9), Simd(2)
    simd_op_f32(or) Simd(9), Simd(3)

    simd_broadcast_m32 Simd(4), [r8 + 96 + 16]
    simd_broadcast_m32 Simd(5), [r8 + 96 + 20]

    simd_op_f32(cmpgt) Simd(4), Simd(10)
    simd_op_f32(cmplt) Simd(5), Simd(11)

    simd_op_f32(or) Simd(9), Simd(4)
    simd_op_f32(or) Simd(9), Simd(5)
%endif

    ; r9 was used as a parameter, so it's safe to reuse it here
    simd_op_f32(movmsk) r9, Simd(9)

    add         rcx, 64
    shl         r9, cl
    or          rax, r9
    add         rcx,-64 + SIMD_SIZE

    jz          .storebits_and_loop

    ; increment byte offset to the next AoSoA packet of boxes
    add         rdx, SimdByteSize * 6
    jnz         .loop

    ; early out path:
    ; store RAX containing visibility mask for [0-60] boxes
    ; use non-temporal hint (?not necessary if dst will be used right after this function finishes)
    movnti      [rdi], rax
    jmp         .loop_exit

.storebits_and_loop:
    ; regular path:
    ; store RAX containing visibility mask for 64 boxes
    ; use non-temporal hint (?not necessary if dst will be used right after this function finishes)
    movnti      [rdi], rax

    ; increment RDI address and clear RAX
    add         rdi, 8
    xor         rax, rax

    ; reset RCX counter to the default value
    mov         rcx, -64

    ; increment byte offset to the next AoSoA packet of boxes
    add rdx, SimdByteSize * 6
    jnz .loop

.loop_exit:

    ;IACA_END

    EndSrcDstLoop

    SimdReLoadNonVolatileRegistersFromStack

    mov rsp, rbp
    pop rbp

    ret

%endif

; This software is distributed under 2 licenses -- choose whichever you prefer.
; ------------------------------------------------------------------------------
; ALTERNATIVE A - MIT License
;
; Copyright (c) 2022 Pavel Martishevsky
;
; Permission is hereby granted, free of charge, to any person obtaining a copy of
; this software and associated documentation files (the "Software"), to deal in
; the Software without restriction, including without limitation the rights to
; use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
; of the Software, and to permit persons to whom the Software is furnished to do
; so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in all
; copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
; SOFTWARE.
; ------------------------------------------------------------------------------
; ALTERNATIVE B - Public Domain (www.unlicense.org)
;
; This is free and unencumbered software released into the public domain.
; Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
; software, either in source code form or as a compiled binary, for any purpose,
; commercial or non-commercial, and by any means.
; In jurisdictions that recognize copyright laws, the author or authors of this
; software dedicate any and all copyright interest in the software to the public
; domain. We make this dedication for the benefit of the public at large and to
; the detriment of our heirs and successors. We intend this dedication to be an
; overt act of relinquishment in perpetuity of all present and future rights to
; this software under copyright law.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
; ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
; WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
