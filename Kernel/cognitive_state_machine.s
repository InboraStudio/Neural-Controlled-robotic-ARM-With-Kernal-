; cognitive_state_machine.s
; Hierarchical state machine for autonomous decision making
; Implements goal-oriented behavior through memory addresses

.section .text

; state_transition: evaluate state transitions based on context
; r25:r24 = current_state_ptr, r23:r22 = context_vector_ptr
.global state_transition
state_transition:
    push r16
    push r17
    
    mov X, r24
    ld r16, X
    
    mov Z, r22
    ldi r26, 8
    clr r17
    
eval_context:
    ld r0, Z+
    or r17, r0
    dec r26
    brne eval_context
    
    cpi r17, 0x00
    breq neutral_state
    
    mov X, r24
    ldi r16, 0x02
    st X, r16
    jmp trans_done
    
neutral_state:
    mov X, r24
    clr r16
    st X, r16
    
trans_done:
    pop r17
    pop r16
    ret

; goal_pursuit: compute gradient toward goal
; r25:r24 = current_position_ptr, r23:r22 = goal_position_ptr
.global goal_pursuit
goal_pursuit:
    mov X, r24
    mov Z, r22
    
    ld r0, X
    ld r1, Z
    
    sub r1, r0
    
    mov r24, r1
    ret

; exploration_exploitation: balance between curiosity and exploitation
; r24 = exploitation_value, r25 = exploration_bonus
.global exploration_exploitation
exploration_exploitation:
    mov r0, r25
    mul r0, 0x20
    mov r0, r0
    
    add r24, r0
    
    ret

; emotional_valence: compute emotional state
; r25:r24 = reward_ptr, r23 = prev_reward
.global emotional_valence
emotional_valence:
    mov X, r24
    ld r0, X
    
    sub r0, r23
    
    tst r0
    brpl positive_emotion
    
    ldi r24, 0x00
    jmp emotion_done
    
positive_emotion:
    ldi r24, 0xFF
    
emotion_done:
    ret
