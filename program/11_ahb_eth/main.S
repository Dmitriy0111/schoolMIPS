#define MAX_ADDR_OFFSET      0x20
#define ETH_START 			 0x50000000

# ETH_DEST_ADDR broadcast (MAC_ADDR)
#define ETH_DEST_0			 0xFF
#define ETH_DEST_1			 0xFF
#define ETH_DEST_2			 0xFF
#define ETH_DEST_3			 0xFF
#define ETH_DEST_4			 0xFF
#define ETH_DEST_5			 0xFF

# ETH_SOUR_ADDR custom    (MAC_ADDR)
#define ETH_SOUR_0			 0x00
#define ETH_SOUR_1			 0x12
#define ETH_SOUR_2			 0x34
#define ETH_SOUR_3			 0x56
#define ETH_SOUR_4			 0x78
#define ETH_SOUR_5			 0x90

# ETH_TYPE
#define ETH_TYPE_0			 0x08
#define ETH_TYPE_1			 0x00

# ETH_IP_HEADER
#define ETH_IP_HEADER_0	     0x45
#define ETH_IP_HEADER_1		 0x00
#define ETH_IP_HEADER_2	     0x00
#define ETH_IP_HEADER_3		 0x2E
#define ETH_IP_HEADER_4	     0xB3
#define ETH_IP_HEADER_5		 0xFE
#define ETH_IP_HEADER_6	     0x00
#define ETH_IP_HEADER_7		 0x00
#define ETH_IP_HEADER_8		 0x80

# ETH_IP_PROTOCOL (UDP)
#define ETH_IP_PROTOCOL_0	 0x11

# ETH_IP_CHECKSUM
#define ETH_IP_CHECKSUM_0	 0x05
#define ETH_IP_CHECKSUM_1	 0x40

# ETH_IP_SOURCE			(192.168.0.44)
#define ETH_IP_SOURCE_0	     0xC0
#define ETH_IP_SOURCE_1	     0xA8
#define ETH_IP_SOURCE_2	     0x00
#define ETH_IP_SOURCE_3	     0x2C

# ETH_IP_DESTINATION	(192.168.0.04)
#define ETH_IP_DESTINATION_0 0xC0
#define ETH_IP_DESTINATION_1 0xA8
#define ETH_IP_DESTINATION_2 0x00
#define ETH_IP_DESTINATION_3 0x04

# ETH_IP_S_PORT      	(192.168.0.44:1024)
#define ETH_IP_S_PORT_0 	 0x04
#define ETH_IP_S_PORT_1  	 0x00

# ETH_IP_D_PORT      	(192.168.0.04:1024)
#define ETH_IP_D_PORT_0 	 0x04
#define ETH_IP_D_PORT_1  	 0x00

# ETH_PAYLOAD_LENGTH    (18)
#define ETH_PAYLOAD_LENGTH_0 0x00
#define ETH_PAYLOAD_LENGTH_1 0x1A

# ETH_UDP_CHECKSUM      
#define ETH_UDP_CHECKSUM_0   0x2D
#define ETH_UDP_CHECKSUM_1   0xE8

# ETH_PAYLOAD         (Hello World!!!!!!!)  
#define ETH_PAYLOAD_0        0x48
#define ETH_PAYLOAD_1        0x65
#define ETH_PAYLOAD_2        0x6C
#define ETH_PAYLOAD_3        0x6C
#define ETH_PAYLOAD_4        0x6F
#define ETH_PAYLOAD_5        0x20
#define ETH_PAYLOAD_6        0x77
#define ETH_PAYLOAD_7        0x6F
#define ETH_PAYLOAD_8        0x72
#define ETH_PAYLOAD_9        0x6C
#define ETH_PAYLOAD_10       0x64
#define ETH_PAYLOAD_11       0x21
#define ETH_PAYLOAD_12       0x21
#define ETH_PAYLOAD_13       0x21
#define ETH_PAYLOAD_14       0x21
#define ETH_PAYLOAD_15       0x21
#define ETH_PAYLOAD_16       0x21
#define ETH_PAYLOAD_17       0x21


            .text

init:       li      $t0, ETH_START       # int i = 0;
            move    $v0, $0              # int a = 0;

generate_eth_frame:
            # ETH_DEST_ADDR broadcast (MAC_ADDR)
			li		$v0, ETH_DEST_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_DEST_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_DEST_2
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_DEST_3
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_DEST_4
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_DEST_5
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_SOUR_ADDR custom    (MAC_ADDR)
			li		$v0, ETH_SOUR_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_SOUR_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_SOUR_2
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_SOUR_3
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_SOUR_4
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_SOUR_5
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_TYPE
			li		$v0, ETH_TYPE_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_TYPE_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_IP_HEADER
            li		$v0, ETH_IP_HEADER_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_2
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_3
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_4
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_5
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_6
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_7
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_HEADER_8
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_IP_PROTOCOL (UDP)
            li		$v0, ETH_IP_PROTOCOL_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_IP_CHECKSUM
			li		$v0, ETH_IP_CHECKSUM_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_CHECKSUM_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_IP_SOURCE			(192.168.0.44)
			li		$v0, ETH_IP_SOURCE_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_SOURCE_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_SOURCE_2
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_SOURCE_3
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_IP_DESTINATION	(192.168.0.04)
			li		$v0, ETH_IP_DESTINATION_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_DESTINATION_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_DESTINATION_2
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_DESTINATION_3
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_IP_S_PORT      	(192.168.0.44:1024)
			li		$v0, ETH_IP_S_PORT_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_S_PORT_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_IP_D_PORT      	(192.168.0.04:1024)
			li		$v0, ETH_IP_D_PORT_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_IP_D_PORT_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_PAYLOAD_LENGTH      	
			li		$v0, ETH_PAYLOAD_LENGTH_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_LENGTH_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_UDP_CHECKSUM
			li		$v0, ETH_UDP_CHECKSUM_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_UDP_CHECKSUM_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            # ETH_PAYLOAD
            li		$v0, ETH_PAYLOAD_0
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_1
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_2
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_3
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_4
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_5
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_6
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_7
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_8
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_9
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_10
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_11
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_12
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_13
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_14
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_15
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_16
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, ETH_PAYLOAD_17
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
			
			# ETH Transmit
			addiu   $t0, $t0, 4         # i += 4;
			li		$v0, 0x01
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            
            # Waiting for the end of the transfer
            li      $v1, 0x00
wait1:
            lw      $v0, 0x0 ($t0)      # a = mem[i];
            bne     $v0, $v1, wait1      # if (i != max) goto write
            li		$v0, 0x01
            sw      $v0, 0x0 ($t0)      # mem[i] = a;

            b       wait1                 # while(1);


            # Waiting for the end of the transfer
            li      $v1, 0x00
wait2:
            lw      $v0, 0x0 ($t0)      # a = mem[i];
            bne     $v0, $v1, wait2      # if (i != max) goto write
            li		$v0, 0x02
            sw      $v0, 0x0 ($t0)      # mem[i] = a;
            li		$v0, 0x01
            sw      $v0, 0x0 ($t0)      # mem[i] = a;

end:        b       end                 # while(1);

