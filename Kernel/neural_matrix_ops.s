; neural_matrix_ops.s
; Deep neural network matrix operations in pure assembly
; Implements low-level tensor manipulations and weight calculations

.section .text

; neural_matrix_multiply: multiply two weight matrices
; r25:r24 = matrix_a_ptr, r23:r22 = matrix_b_ptr, r21:r20 = result_ptr
.global neural_matrix_multiply
neural_matrix_multiply:
    clr r0
    clr r1
    clr r2
    clr r3
    
    ldi r16, 0x08
matrix_loop:
    ld r4, X+
    ld r5, Z+  
    
    mul r4, r5
    add r0, r0
    adc r1, r1
    
    dec r16
    brne matrix_loop
    
    mov X, r20
    st X+, r0
    st X, r1
    ret

; sigmoid_approximation: fast sigmoid using bit operations
; r24 = input_value
.global sigmoid_approximation
sigmoid_approximation:
    mov r0, r24
    lsr r0
    lsr r0
    lsr r0
    
    ldi r25, 0x80
    add r0, r25
    
    mov r24, r0
    ret

; backprop_gradient: compute gradient for backpropagation
; r24 = error, r25 = activation
.global backprop_gradient
backprop_gradient:
    mul r24, r25
    
    mov r0, r0
    swap r0
    
    ret

; weight_update: update network weights with learning rate
; r24 = old_weight, r25 = gradient, r23 = learning_rate
.global weight_update
weight_update:
    mul r25, r23
    mov r0, r0
    
    sub r24, r0
    
    ret
