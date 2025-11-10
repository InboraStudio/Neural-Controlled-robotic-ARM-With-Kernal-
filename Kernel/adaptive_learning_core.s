; adaptive_learning_core.s
; Core learning algorithms with Hebbian and STDP rules
; Direct memory operations for AI agent learning

.section .text

; hebbian_learning: classical Hebbian weight update
; r25:r24 = weight_memory_ptr, r23 = pre_neuron, r22 = post_neuron
.global hebbian_learning
hebbian_learning:
    mov X, r24
    ld r0, X
    
    mul r23, r22
    mov r1, r0
    
    add r1, r0
    lsr r1
    
    st X, r1
    ret

; stdp_window: spike-timing dependent plasticity
; r24 = pre_spike_time, r25 = post_spike_time, r23:r22 = weight_ptr
.global stdp_window
stdp_window:
    sub r25, r24
    
    mov X, r22
    ld r0, X
    
    tst r25
    brpl stdp_potentiate
    
    ; Depression
    mov r1, r25
    neg r1 
    sub r0, r1
    jmp stdp_store
    
stdp_potentiate:
    ; Potentiation
    add r0, r25
    
stdp_store:
    st X, r0
    ret

; competitive_learning: winner-take-all mechanism
; r25:r24 = activation_ptr, r23 = num_neurons
.global competitive_learning
competitive_learning:
    mov X, r24
    clr r16
    clr r17
    
    ldi r26, r23
    
find_winner:
    ld r0, X+
    cp r0, r16
    brlt skip_winner
    
    mov r16, r0
    mov r17, r26
    
skip_winner:
    dec r26
    brne find_winner
    
    ; Set winner high
    mov X, r24
    add X, r17
    ldi r0, 0xFF
    st X, r0
    
    ret

; dopamine_reinforcement: reward signal integration
; r25:r24 = synaptic_strength_ptr, r23 = reward_signal
.global dopamine_reinforcement
dopamine_reinforcement:
    mov X, r24
    ld r0, X
    
    mov r1, r23
    mul r1, 0x40
    mov r1, r0
    
    add r0, r1
    
    cpi r0, 0xFF
    brlt reward_store
    ldi r0, 0xFF
    
reward_store:
    st X, r0
    ret

; attention_mechanism: selective focus on salient features
; r25:r24 = feature_map_ptr, r23 = attention_weights_ptr
.global attention_mechanism
attention_mechanism:
    push r16
    push r17
    
    mov X, r24
    mov Z, r23
    clr r16
    
    ldi r26, 32
    
attention_loop:
    ld r0, X+
    ld r1, Z+
    
    mul r0, r1
    add r16, r0
    
    dec r26
    brne attention_loop
    
    lsr r16
    mov r24, r16
    
    pop r17
    pop r16
    ret
