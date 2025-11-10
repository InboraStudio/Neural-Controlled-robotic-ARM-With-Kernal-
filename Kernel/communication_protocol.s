; communication_protocol.s
; CAN bus, I2C, SPI communication with explicit memory addressing
; Direct hardware register control for multi-protocol comm

.section .text

; CAN Bus controller: 0x2000 - 0x202F
.equ CAN_CTRL_REG,      0x2000
.equ CAN_STATUS_REG,    0x2001
.equ CAN_BAUD_H,        0x2002
.equ CAN_BAUD_L,        0x2003
.equ CAN_TX_ID_H,       0x2004
.equ CAN_TX_ID_L,       0x2005
.equ CAN_TX_LEN,        0x2006
.equ CAN_TX_DATA_0,     0x2007
.equ CAN_TX_DATA_1,     0x2008
.equ CAN_TX_DATA_2,     0x2009
.equ CAN_TX_DATA_3,     0x200A
.equ CAN_TX_DATA_4,     0x200B
.equ CAN_TX_DATA_5,     0x200C
.equ CAN_TX_DATA_6,     0x200D
.equ CAN_TX_DATA_7,     0x200E

.equ CAN_RX_ID_H,       0x200F
.equ CAN_RX_ID_L,       0x2010
.equ CAN_RX_LEN,        0x2011
.equ CAN_RX_DATA_0,     0x2012
.equ CAN_RX_DATA_1,     0x2013
.equ CAN_RX_DATA_2,     0x2014
.equ CAN_RX_DATA_3,     0x2015
.equ CAN_RX_DATA_4,     0x2016
.equ CAN_RX_DATA_5,     0x2017
.equ CAN_RX_DATA_6,     0x2018
.equ CAN_RX_DATA_7,     0x2019

; I2C Master: 0x2100 - 0x211F
.equ I2C_CTRL,          0x2100
.equ I2C_STATUS,        0x2101
.equ I2C_ADDR,          0x2102
.equ I2C_DATA,          0x2103
.equ I2C_SPEED,         0x2104

; SPI Master: 0x2200 - 0x221F
.equ SPI_CTRL,          0x2200
.equ SPI_STATUS,        0x2201
.equ SPI_DATA,          0x2202
.equ SPI_CLOCK_DIV,     0x2203

; UART: 0x2300 - 0x230F
.equ UART_CTRL,         0x2300
.equ UART_STATUS,       0x2301
.equ UART_BAUD_H,       0x2302
.equ UART_BAUD_L,       0x2303
.equ UART_TX_DATA,      0x2304
.equ UART_RX_DATA,      0x2305

; global can_transmit_frame
.global can_transmit_frame
can_transmit_frame:
    ; r25:r24 = CAN ID, r23 = data length, r22 = data pointer
    
    ldi r26, CAN_TX_ID_H
    st X+, r25
    st X+, r24
    
    ldi r26, CAN_TX_LEN
    st X, r23
    
    mov X, r22
    
    ldi r26, CAN_TX_DATA_0
    ldi r20, 0
    
can_tx_loop:
    cpi r20, r23
    brge can_tx_send
    
    ld r0, X+
    st Z+, r0
    inc r20
    jmp can_tx_loop
    
can_tx_send:
    ldi r26, CAN_CTRL_REG
    ldi r24, 0x01
    st X, r24
    
    ret

; global can_receive_frame
.global can_receive_frame
can_receive_frame:
    ; r25:r24 = buffer address
    
    ldi r26, CAN_RX_LEN
    ld r20, X
    
    mov X, r24
    mov r16, r25
    
    ldi r26, CAN_RX_DATA_0
    ldi r23, 0
    
can_rx_loop:
    cpi r23, r20
    brge can_rx_done
    
    ld r0, Z+
    st X+, r0
    inc r23
    jmp can_rx_loop
    
can_rx_done:
    ret

; global i2c_write
.global i2c_write
i2c_write:
    ; r25 = slave address, r24 = data byte
    
    ldi r26, I2C_ADDR
    st X, r25
    
    ldi r26, I2C_DATA
    st X, r24
    
    ldi r26, I2C_CTRL
    ldi r24, 0x01
    st X, r24
    
    ret

; global i2c_read
.global i2c_read
i2c_read:
    ; r24 = slave address
    ; Returns data in r24
    
    ldi r26, I2C_ADDR
    st X, r24
    
    ldi r26, I2C_CTRL
    ldi r24, 0x02
    st X, r24
    
    ; Wait for completion
    ldi r26, 100
i2c_wait:
    dec r26
    brne i2c_wait
    
    ldi r26, I2C_DATA
    ld r24, X
    ret

; global spi_transmit
.global spi_transmit
spi_transmit:
    ; r24 = byte to transmit
    
    ldi r26, SPI_DATA
    st X, r24
    
    ret

; global spi_receive
.global spi_receive
spi_receive:
    ; Returns received byte in r24
    
    ldi r26, SPI_DATA
    ld r24, X
    ret

; global uart_send_byte
.global uart_send_byte
uart_send_byte:
    ; r24 = byte to send
    
    ldi r26, UART_TX_DATA
    st X, r24
    
    ret

; global uart_receive_byte
.global uart_receive_byte
uart_receive_byte:
    ; Returns received byte in r24
    
    ldi r26, UART_RX_DATA
    ld r24, X
    ret

; global init_can_bus
.global init_can_bus
init_can_bus:
    ; Initialize CAN bus at 500kbps
    
    ldi r26, CAN_CTRL_REG
    ldi r24, 0x00
    st X, r24
    
    ldi r26, CAN_BAUD_H
    ldi r24, 0x01
    st X+, r24
    
    ldi r24, 0x2E
    st X, r24
    
    ; Enable CAN
    ldi r26, CAN_CTRL_REG
    ldi r24, 0x01
    st X, r24
    
    ret

; global init_i2c
.global init_i2c
init_i2c:
    ; Initialize I2C at 400kHz
    
    ldi r26, I2C_SPEED
    ldi r24, 0x0C
    st X, r24
    
    ldi r26, I2C_CTRL
    ldi r24, 0x01
    st X, r24
    
    ret

; global init_uart
.global init_uart
init_uart:
    ; Initialize UART at 115200 bps
    
    ldi r26, UART_BAUD_H
    clr r24
    st X+, r24
    
    ldi r24, 0x08
    st X, r24
    
    ldi r26, UART_CTRL
    ldi r24, 0x18
    st X, r24
    
    ret
