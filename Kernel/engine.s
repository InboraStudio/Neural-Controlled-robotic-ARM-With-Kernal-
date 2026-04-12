.section .text

.global tensor_convolution_3d
tensor_convolution_3d:
    push r16
    push r17
    push r18
    push r19

    ; Setup X = input pointer
    mov r26, r24
    mov r27, r25

    ; Setup Z = kernel pointer
    mov r30, r22
    mov r31, r23

    clr r16        ; acc low
    clr r17        ; acc high

    ldi r18, 9     ; loop counter

conv_kernel_loop:
    ld r0, X+      ; input
    ld r1, Z+      ; kernel

    mul r0, r1     ; result → r1:r0

    add r16, r0
    adc r17, r1

    clr r1         ; REQUIRED after MUL (ABI rule)

    dec r18
    brne conv_kernel_loop

    ; Store result (output ptr in r21:r20)
    mov r26, r20
    mov r27, r21

    st X+, r16
    st X,  r17

    pop r19
    pop r18
    pop r17
    pop r16
    ret

.global activation_relu
activation_relu:
    sbrc r24, 7     ; if negative (bit7=1)
    clr r24
    ret

.global activation_tanh
activation_tanh:
    mov r0, r24
    mul r0, r24     ; x²
    mov r18, r0

    mul r18, r24    ; x³

    lsr r1          ; divide by 128 (>>7)
    lsr r1
    lsr r1
    lsr r1
    lsr r1
    lsr r1
    lsr r1

    sub r24, r1     ; x - x³/128

    clr r1
    ret


.global layer_batch_norm
layer_batch_norm:
    ; X = data pointer
    mov r26, r24
    mov r27, r25

    ldi r18, 16

norm_loop:
    ld r0, X

    sub r0, r23     ; x - mean

    ; fast variance approx (>>2 instead of loop)
    mov r19, r22
    lsr r19
    lsr r19

    add r0, r19

    st X+, r0

    dec r18
    brne norm_loop

    ret
