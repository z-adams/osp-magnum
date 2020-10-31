.data

vec0 real8 0.0, 0.0, 0.0, 0.0
vec1 real8 1.0, 1.0, 1.0, 1.0

Sources struct
    v_xs dq ??
    v_ys dq ??
    v_zs dq ??
    v_ms dq ??
Sources ends

.code
;                   input registers:                  rcx           rdx           r8
; function signature: fast_function_update(__m256d ownPos, Sources* srcs, size_t nSrcs)
fast_linear_update proc
    ;  Params:
    ;   - RCX    OwnPos ptr
    ;   - RDX    Sources ptr
    ;   - R8     Loop limit
    ;  Setup:
    ;   - RAX    Loop counter
    ;   - YMM0   Net acceleration accumulator
    ;   - YMM1   Own pos Xs
    ;   - YMM2   Own pos Ys
    ;   - YMM3   Own pos Zs
    ;   - R11    X vals ptr
    ;   - R12    Y vals ptr
    ;   - R13    Z vals ptr
    ;   - R14    Masses ptr

    ; R12-15, XMM6-15 are callee-saved
    ; Save the ones we use
    sub rsp, 16*5
    ;; TODO: alignment to use movdqa
    movdqu xmmword ptr [rsp], xmm6  ; scratch
    movdqu xmmword ptr [rsp+16], xmm11 ; loop input data
    movdqu xmmword ptr [rsp+32], xmm12 ;
    movdqu xmmword ptr [rsp+48], xmm13 ;
    movdqu xmmword ptr [rsp+64], xmm14 ;

    ; Net acceleration
    vmovapd ymm0, ymmword ptr [vec0]

    ; Own pos [x, y, z, 0], broadcasted into x, y, z registers
    vbroadcastsd ymm1, real8 ptr [rcx+24]
    vbroadcastsd ymm2, real8 ptr [rcx+16]
    vbroadcastsd ymm3, real8 ptr [rcx+8]

    mov rcx, rdx
    ; # RCX becomes sources ptr
    ; # RDX becomes scratch

    ; Compute number of loops (nSources / 4), store in RDX
    ; Set up division
    mov rdx, 0
    mov rax, r8
    mov r8, 4
    div r8
    mov rdx, rax
    ; # RDX becomes num loops to free r8 for array ptrs
    ; This way we can avoid using R12-15 (callee saved)

    ; Vector arrays
    mov r8,  qword ptr [rcx]        ; X values   try struct?
    mov r9,  qword ptr [rcx+8]      ; Y values
    mov r10, qword ptr [rcx+16]     ; Z values
    mov r11, qword ptr [rcx+24]     ; Mass values
    ; # R8-R11 become vector array iterators

    mov rax, 0
    ; # RAX becomes loop counter
LoopStart:

    ; Grab 4 X values
    vmovapd ymm11, ymmword ptr [r8]
    ; Grab 4 Y values
    vmovapd ymm12, ymmword ptr [r9]
    ; Grab 4 Z values
    vmovapd ymm13, ymmword ptr [r10]
    ; Grab 4 masses
    vmovapd ymm14, ymmword ptr [r11]
    ; # YMM14  Current masses


    ; Subtract src.x - ownPos.x
    vsubpd ymm11, ymm11, ymm1
    ; Subtract src.y - ownPos.y
    vsubpd ymm12, ymm12, ymm2
    ; Subtract src.z - ownPos.z
    vsubpd ymm13, ymm13, ymm3
    ; # YMM11  Rel. X vals \
    ; # YMM12  Rel. Y vals  } Rel pos vectors
    ; # YMM13  Rel. Z vals /

    ; ### Vector normalization ###

    ; ## Denominators ##
    ; Square components, store in ymm4-6
    vmulpd ymm4, ymm11, ymm11 ; X1*X1, X2*X2, X3*X3, X4*X4
    vmulpd ymm5, ymm12, ymm12 ; Y1*Y1, Y2*Y2, Y3*Y3, Y4*Y4
    vmulpd ymm6, ymm13, ymm13 ; Z1*Z1, Z2*Z2, Z3*Z3, Z4*Z4
    ; # YMM4   X*X vals
    ; # YMM5   Y*Y vals
    ; # YMM6   Z*Z vals

    ; Vertical sum to get norm squared
    vaddpd ymm4, ymm4, ymm5 ; X + Y
    vaddpd ymm4, ymm4, ymm6 ; (X + Y) + Z
    ; # YMM4 becomes (X*X + Y*Y + Z*Z)1, (2), (3), (4)

    ; YMM6 stores norm squared of the vectors, compute norm and invert
    vsqrtpd ymm6, ymm4
    vmovapd ymm5, ymmword ptr [vec1]
    vdivpd ymm6, ymm5, ymm6
    ; # YMM6 becomes 1/norm, YMM4 becomes available

    ; ### Force summation ###
    
    ; Compute gravity coefficients (mass / denom) (mutates denominators)
    vdivpd ymm4, ymm14, ymm4
    ; Factor in normalization
    vmulpd ymm6, ymm4, ymm6
    ; # YMM6 becomes full gravity coefficients

    ; Multiply components by coefficients (1/norm) * (mass / denom)
    vmulpd ymm11, ymm11, ymm6
    vmulpd ymm12, ymm12, ymm6
    vmulpd ymm13, ymm13, ymm6
    ; # YMM6 becomes available

    ; Registers:
    ; YMM11: [F4.x, F3.x, F2.x, F1.x]
    ; YMM12: [F4.y, F3.y, F2.y, F1.y]
    ; YMM13: [F4.z, F3.z, F2.z, F1.z]

    ; Need: [F.x, F.y, F.z, 0]

    ; Horizontal sum into net force
    vhaddpd ymm4, ymm11, ymm12
    ; # YMM4 becomes [y3+y4, x3+x4, y1+y2, x1+x2]
    vpermpd ymm4, ymm4, 01110010b  ; permute 3,2,1,0 -> 1,3,0,2
    ; # YMM4 becomes [y1+y2, y3+y4, x1+x2, x3+x4]

    vhaddpd ymm6, ymm13, ymm13
    ; # YMM6 becomes [z3+z4, z3+z4, z1+z2, z1+z2]
    vpermpd ymm6, ymm6, 00100111b  ; permute 3,2,1,0 -> 0,2,1,3
    ; # YMM6 becomes [z1+z2, z3+z4, z1+z2, z3+z4]
    ;vblendpd ymm6, ymm4, ymm6, 0011b
    vperm2f128 ymm6, ymm4, ymm6, 00010b
    ; # YMM6 becomes [x1+x2, x3+x4, z1+z2, z3+z4]

    vhaddpd ymm6, ymm4, ymm6
    ;      YMM4 [y1+y2, y3+y4, x1+x2, x3+x4]
    ; HADD YMM6 [x1+x2, x3+x4, z1+z2, z3+z4]
    ;    result [X1234, Y1234, Z1234, X1234]
    ; # YMM6 becomes [X, Y, Z, X]

    ; Perform final acceleration add
    vaddpd ymm0, ymm0, ymm6

    ; Incr counter & test loop condition
    inc rax
    cmp rax, rdx
    jae LoopEscape

    add r8, 32
    add r9, 32
    add r10, 32
    add r11, 32
    jmp LoopStart

LoopEscape:
    ; pop saved registers
    
    ;; TODO: alignment to use movdqa
    movdqu xmm6, xmmword ptr [rsp]  ; scratch
    movdqu xmm11, xmmword ptr [rsp+16] ; loop input data
    movdqu xmm12, xmmword ptr [rsp+32] ;
    movdqu xmm13, xmmword ptr [rsp+48] ;
    movdqu xmm14, xmmword ptr [rsp+64] ;
    
    add rsp, 16*5

    ; Acceleration already in YMM0
    ret
fast_linear_update endp
end